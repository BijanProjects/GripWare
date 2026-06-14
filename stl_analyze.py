"""stl_analyze.py — extract geometry from the arm's STL parts.

Reads every .stl in a folder (default ../3D_Files) and reports, per part:
    - triangle count, bounding box + size, centroid, solid volume, surface area
    - principal axes (PCA) and the extent of the part along each
    - detected cylindrical holes / bosses (axis, centre, radius, length) via a
      normal-constrained RANSAC circle fit -- these locate servo shafts and
      pivot holes, which give the true pivot-to-pivot link lengths the IK needs.

Pure binary-STL parser + numpy. Run:
    python stl_analyze.py [folder]
Writes a Markdown summary to STL_REPORT.md and prints it.
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np


def load_binary_stl(path: Path):
    """Return (normals[N,3], tris[N,3,3]) from a binary STL."""
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        data = f.read(n * 50)
    if len(data) < n * 50:
        raise ValueError("truncated STL")
    rec = np.frombuffer(data, dtype=np.uint8).reshape(n, 50)
    floats = rec[:, :48].copy().view("<f4").reshape(n, 4, 3)
    normals = floats[:, 0, :]
    tris = floats[:, 1:, :]
    return normals.astype(np.float64), tris.astype(np.float64)


def mesh_stats(tris):
    """bbox, size, vertex-centroid, signed volume (mm^3), surface area (mm^2)."""
    v = tris.reshape(-1, 3)
    lo = v.min(axis=0)
    hi = v.max(axis=0)
    size = hi - lo
    # surface area = sum of triangle areas
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    cross = np.cross(b - a, c - a)
    area = 0.5 * np.linalg.norm(cross, axis=1).sum()
    # signed volume via divergence theorem (sum of tetra (origin, a,b,c))
    vol = np.abs(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)
    return lo, hi, size, v.mean(axis=0), vol, area


def principal_axes(tris):
    """PCA on unique-ish vertices -> (eigvals, eigvecs cols, extents along each)."""
    v = tris.reshape(-1, 3)
    c = v.mean(axis=0)
    cov = np.cov((v - c).T)
    w, vecs = np.linalg.eigh(cov)
    order = np.argsort(w)[::-1]          # largest variance first
    vecs = vecs[:, order]
    proj = (v - c) @ vecs
    extents = proj.max(axis=0) - proj.min(axis=0)
    return w[order], vecs, extents, c


def _circle_from_two_normals(p1, m1, p2, m2):
    """Centre lying on both normal lines p_i + s*m_i. Returns centre or None."""
    A = np.array([[m1[0], -m2[0]], [m1[1], -m2[1]]])
    det = A[0, 0] * A[1, 1] - A[0, 1] * A[1, 0]
    if abs(det) < 1e-6:
        return None
    s = np.linalg.solve(A, p2 - p1)
    return p1 + s[0] * m1


def detect_holes(normals, tris, axis, min_r=1.5, max_r=60.0,
                 tol=0.6, iters=1500, min_support=40, max_circles=6, rng=None):
    """RANSAC circle fit on facets whose walls run parallel to `axis` (0/1/2).

    Returns list of dicts: {radius, centre(3d), axis, length, support}.
    A hole/boss is a cylinder: its wall facets have normals perpendicular to
    the cylinder axis and pointing along the radius.
    """
    if rng is None:
        rng = np.random.default_rng(0)
    e = np.eye(3)[axis]
    nlen = np.linalg.norm(normals, axis=1)
    good = nlen > 1e-9
    nrm = np.zeros_like(normals)
    nrm[good] = normals[good] / nlen[good, None]
    wall = good & (np.abs(nrm @ e) < 0.25)
    idx = np.where(wall)[0]
    if len(idx) < min_support:
        return []

    keep = [k for k in range(3) if k != axis]
    cent = tris.mean(axis=1)                  # facet centroids (N,3)
    P = cent[idx][:, keep]                    # 2D positions
    M = nrm[idx][:, keep]                      # 2D normals
    Mn = np.linalg.norm(M, axis=1)
    ok = Mn > 1e-6
    P, M, idx = P[ok], M[ok] / Mn[ok, None], idx[ok]
    if len(P) < min_support:
        return []

    circles = []
    avail = np.ones(len(P), dtype=bool)
    for _ in range(max_circles):
        ai = np.where(avail)[0]
        if len(ai) < min_support:
            break
        best_inl, best_c, best_r = None, None, None
        for _ in range(iters):
            i, j = rng.choice(ai, 2, replace=False)
            c = _circle_from_two_normals(P[i], M[i], P[j], M[j])
            if c is None:
                continue
            r = 0.5 * (np.linalg.norm(c - P[i]) + np.linalg.norm(c - P[j]))
            if not (min_r <= r <= max_r):
                continue
            d = np.linalg.norm(P[ai] - c, axis=1)
            radial = (P[ai] - c) / np.clip(d, 1e-9, None)[:, None]
            align = np.abs(np.einsum("ij,ij->i", radial, M[ai]))
            inl = ai[(np.abs(d - r) < tol) & (align > 0.85)]
            if best_inl is None or len(inl) > len(best_inl):
                best_inl, best_c, best_r = inl, c, r
        if best_inl is None or len(best_inl) < min_support:
            break
        # refine radius from inliers; record 3D centre + length along axis
        d = np.linalg.norm(P[best_inl] - best_c, axis=1)
        r = float(d.mean())
        along = cent[idx[best_inl]][:, axis]
        c3 = np.zeros(3)
        c3[keep] = best_c
        c3[axis] = float(along.mean())
        circles.append({"radius": r, "centre": c3, "axis": axis,
                        "length": float(along.max() - along.min()),
                        "support": int(len(best_inl))})
        avail[best_inl] = False
    return circles


AXIS = "XYZ"


def analyze(path: Path) -> str:
    normals, tris = load_binary_stl(path)
    lo, hi, size, cen, vol, area = mesh_stats(tris)
    w, vecs, extents, _ = principal_axes(tris)

    out = [f"### {path.name}", ""]
    out.append(f"- triangles: **{len(tris)}**")
    out.append(f"- bbox size (X,Y,Z): **{size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} mm**")
    out.append(f"- bbox min / max: ({lo[0]:.2f}, {lo[1]:.2f}, {lo[2]:.2f}) / "
               f"({hi[0]:.2f}, {hi[1]:.2f}, {hi[2]:.2f})")
    out.append(f"- centroid: ({cen[0]:.2f}, {cen[1]:.2f}, {cen[2]:.2f}) mm")
    out.append(f"- solid volume: **{vol/1000.0:.2f} cm^3**   surface area: {area/100.0:.2f} cm^2")
    out.append(f"- principal extents (PCA, long->short): "
               f"{extents[0]:.1f}, {extents[1]:.1f}, {extents[2]:.1f} mm")

    holes = []
    for ax in range(3):
        holes += detect_holes(normals, tris, ax)
    holes.sort(key=lambda h: -h["support"])
    if holes:
        out.append("- detected cylinders (candidate shaft/pivot/mount holes):")
        for h in holes[:8]:
            c = h["centre"]
            out.append(f"    - axis **{AXIS[h['axis']]}**, r={h['radius']:.2f} mm, "
                       f"len={h['length']:.1f} mm, centre=({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}), "
                       f"support={h['support']}")
    else:
        out.append("- detected cylinders: none above support threshold")
    out.append("")
    return "\n".join(out), holes


def main():
    folder = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("../3D_Files")
    files = sorted(folder.glob("*.stl"))
    if not files:
        raise SystemExit(f"no STL files in {folder}")

    report = ["# STL geometry report", "",
              f"Source: `{folder}`  ({len(files)} parts).  Units: millimetres.",
              "Cylinder detection is a normal-constrained RANSAC fit; treat radii",
              "as approximate. Pivot-to-pivot link length = distance between the",
              "two end shaft holes along a part's long axis.", ""]
    all_holes = {}
    for f in files:
        try:
            text, holes = analyze(f)
            report.append(text)
            all_holes[f.name] = holes
        except Exception as exc:  # noqa: BLE001
            report.append(f"### {f.name}\n- ERROR: {exc}\n")

    # Derive pivot-to-pivot estimates for the two main links. A pivot is a
    # TRANSVERSE shaft hole (axis != the part's long axis) with a plausible
    # boss radius; the link length is the span between the two end pivots.
    def link_len(name):
        hs = all_holes.get(name)
        if not hs:
            return None
        _, tris = load_binary_stl(folder / name)
        v = tris.reshape(-1, 3)
        la = int(np.argmax(v.max(0) - v.min(0)))           # long axis
        mid = 0.5 * (v[:, la].max() + v[:, la].min())
        # transverse shaft holes only (axis != long axis, plausible boss radius)
        piv = [h for h in hs if h["axis"] != la and 3.0 <= h["radius"] <= 25.0]
        lo = [h for h in piv if h["centre"][la] < mid]
        hi = [h for h in piv if h["centre"][la] >= mid]
        if not lo or not hi:
            return None
        # dominant shaft in each half = largest radius
        a = max(lo, key=lambda h: h["radius"])
        b = max(hi, key=lambda h: h["radius"])
        return abs(b["centre"][la] - a["centre"][la]), AXIS[la], (a, b)

    report.append("## Derived link lengths (pivot-to-pivot estimates)\n")
    for part, role in [("Alt_Kol.stl", "L1 shoulder->elbow (lower arm)"),
                       ("On_Kol.stl", "L2 elbow->wrist (upper arm)"),
                       ("Bilek.stl", "wrist pivot block")]:
        r = link_len(part)
        if r:
            a, b = r[2]
            report.append(f"- **{part}** ({role}): ~**{r[0]:.1f} mm** along axis {r[1]} "
                          f"(shaft bosses r={a['radius']:.1f}@{a['centre'][AXIS.index(r[1])]:.0f} "
                          f"and r={b['radius']:.1f}@{b['centre'][AXIS.index(r[1])]:.0f})")
        else:
            report.append(f"- **{part}** ({role}): not enough transverse holes; "
                          f"use bbox long-axis as the estimate")
    report.append("")

    text = "\n".join(report)
    Path("STL_REPORT.md").write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
