# STL geometry report

Source: `..\3D_Files`  (20 parts).  Units: millimetres.
Cylinder detection is a normal-constrained RANSAC fit; treat radii
as approximate. Pivot-to-pivot link length = distance between the
two end shaft holes along a part's long axis.

### Alt_Govde.stl

- triangles: **7288**
- bbox size (X,Y,Z): **120.00 x 119.98 x 60.00 mm**
- bbox min / max: (-60.00, -59.99, -10.00) / (60.00, 59.99, 50.00)
- centroid: (15.16, -0.06, 19.81) mm
- solid volume: **158.11 cm^3**   surface area: 415.06 cm^2
- principal extents (PCA, long->short): 125.9, 120.0, 73.2 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **Y**, r=15.24 mm, len=56.3 mm, centre=(42.6, 0.0, 34.9), support=348
    - axis **Z**, r=59.99 mm, len=2.3 mm, centre=(0.4, 0.1, -6.5), support=348
    - axis **X**, r=42.55 mm, len=113.7 mm, centre=(16.7, 13.2, 42.5), support=344
    - axis **Y**, r=2.69 mm, len=67.3 mm, centre=(44.7, -0.6, 2.7), support=293
    - axis **X**, r=59.48 mm, len=74.5 mm, centre=(23.4, -80.1, 39.8), support=286
    - axis **X**, r=29.23 mm, len=99.7 mm, centre=(7.5, 0.2, 21.5), support=282
    - axis **X**, r=52.18 mm, len=87.1 mm, centre=(21.7, -22.9, 43.6), support=267
    - axis **Z**, r=55.64 mm, len=47.5 mm, centre=(49.5, -26.3, 29.8), support=264

### Alt_Kapak.stl

- triangles: **10616**
- bbox size (X,Y,Z): **203.60 x 113.60 x 44.50 mm**
- bbox min / max: (-56.80, -56.80, 0.00) / (146.80, 56.80, 44.50)
- centroid: (34.35, 3.06, 9.76) mm
- solid volume: **104.03 cm^3**   surface area: 599.99 cm^2
- principal extents (PCA, long->short): 207.5, 126.3, 47.6 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **X**, r=2.84 mm, len=136.9 mm, centre=(69.1, -54.4, 8.1), support=561
    - axis **X**, r=2.84 mm, len=136.7 mm, centre=(69.0, 54.0, 8.7), support=555
    - axis **Z**, r=56.93 mm, len=9.2 mm, centre=(0.0, 0.0, 5.1), support=428
    - axis **Z**, r=25.02 mm, len=35.4 mm, centre=(-0.2, -0.2, 16.6), support=417
    - axis **X**, r=43.13 mm, len=192.3 mm, centre=(-3.3, -21.4, -39.9), support=345
    - axis **Z**, r=56.88 mm, len=9.2 mm, centre=(131.7, 0.3, 6.9), support=336
    - axis **X**, r=56.58 mm, len=199.2 mm, centre=(20.9, 14.3, -53.2), support=298
    - axis **Y**, r=21.00 mm, len=105.3 mm, centre=(-18.0, 0.0, 24.2), support=264

### Alt_Kasa.stl

- triangles: **12278**
- bbox size (X,Y,Z): **209.99 x 120.00 x 70.00 mm**
- bbox min / max: (-150.00, -60.00, 0.00) / (59.99, 60.00, 70.00)
- centroid: (-60.21, 1.18, 44.79) mm
- solid volume: **149.53 cm^3**   surface area: 1123.82 cm^2
- principal extents (PCA, long->short): 135.6, 216.7, 77.2 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **X**, r=48.78 mm, len=152.1 mm, centre=(-69.3, 11.2, 40.0), support=1104
    - axis **X**, r=40.93 mm, len=205.5 mm, centre=(-64.2, -19.0, 41.5), support=1021
    - axis **X**, r=41.01 mm, len=204.2 mm, centre=(-58.3, -18.9, 29.2), support=578
    - axis **X**, r=32.54 mm, len=155.2 mm, centre=(-71.3, 27.4, 26.9), support=453
    - axis **Z**, r=57.01 mm, len=64.1 mm, centre=(-0.4, -0.0, 45.0), support=401
    - axis **Z**, r=60.02 mm, len=61.8 mm, centre=(-0.0, -0.4, 43.7), support=356
    - axis **Z**, r=54.99 mm, len=0.9 mm, centre=(-0.3, 0.3, 68.5), support=332
    - axis **Z**, r=47.71 mm, len=64.5 mm, centre=(-78.9, 12.4, 35.3), support=308

### Alt_Kol.stl

- triangles: **4120**
- bbox size (X,Y,Z): **95.00 x 30.00 x 241.77 mm**
- bbox min / max: (-27.50, -15.00, -127.49) / (67.50, 15.00, 114.28)
- centroid: (15.84, 1.64, -17.72) mm
- solid volume: **309.84 cm^3**   surface area: 430.12 cm^2
- principal extents (PCA, long->short): 247.7, 85.8, 38.0 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **Y**, r=10.49 mm, len=27.9 mm, centre=(0.2, 0.0, -100.0), support=288
    - axis **X**, r=17.45 mm, len=71.0 mm, centre=(43.4, -5.2, 93.5), support=180
    - axis **Z**, r=13.70 mm, len=215.1 mm, centre=(-6.7, -1.4, -60.1), support=160
    - axis **X**, r=18.02 mm, len=19.8 mm, centre=(-0.2, 5.8, -105.0), support=144
    - axis **Y**, r=10.49 mm, len=1.0 mm, centre=(47.3, 13.4, 94.4), support=144
    - axis **Z**, r=49.94 mm, len=216.6 mm, centre=(19.2, 35.0, -56.1), support=141
    - axis **X**, r=16.35 mm, len=71.3 mm, centre=(28.0, 0.0, 100.9), support=136
    - axis **Z**, r=13.81 mm, len=215.4 mm, centre=(-5.9, 1.3, -70.5), support=131

### Bilek.stl

- triangles: **2018**
- bbox size (X,Y,Z): **43.00 x 40.00 x 25.00 mm**
- bbox min / max: (-21.50, -20.00, -12.50) / (21.50, 20.00, 12.50)
- centroid: (4.37, 2.46, -0.03) mm
- solid volume: **19.32 cm^3**   surface area: 73.46 cm^2
- principal extents (PCA, long->short): 54.9, 53.1, 25.1 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **X**, r=18.17 mm, len=36.0 mm, centre=(6.4, 1.0, 0.8), support=281
    - axis **Z**, r=18.60 mm, len=18.5 mm, centre=(-3.0, 8.4, -0.1), support=157
    - axis **Z**, r=35.10 mm, len=20.7 mm, centre=(-13.5, 18.1, -0.1), support=156
    - axis **Y**, r=17.61 mm, len=31.8 mm, centre=(-4.3, 6.7, -0.2), support=137
    - axis **X**, r=12.55 mm, len=35.9 mm, centre=(2.9, 7.4, -0.4), support=131
    - axis **Y**, r=17.21 mm, len=32.9 mm, centre=(4.7, 3.9, -3.2), support=124
    - axis **Z**, r=17.49 mm, len=19.0 mm, centre=(6.4, -2.7, 0.1), support=86
    - axis **X**, r=3.49 mm, len=3.7 mm, centre=(-16.0, 7.6, 0.0), support=84

### Disli.stl

- triangles: **1096**
- bbox size (X,Y,Z): **14.73 x 16.86 x 5.00 mm**
- bbox min / max: (-8.48, -8.36, -0.00) / (6.25, 8.50, 5.00)
- centroid: (-2.03, -0.27, 2.51) mm
- solid volume: **0.70 cm^3**   surface area: 7.06 cm^2
- principal extents (PCA, long->short): 17.0, 14.8, 5.2 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **Y**, r=4.51 mm, len=16.0 mm, centre=(-3.8, -0.3, 0.7), support=153
    - axis **Y**, r=3.54 mm, len=14.7 mm, centre=(-4.5, 0.6, 3.3), support=128
    - axis **Z**, r=6.28 mm, len=1.7 mm, centre=(-0.4, -0.1, 2.5), support=114
    - axis **X**, r=4.57 mm, len=11.7 mm, centre=(-1.8, 4.1, 0.7), support=101
    - axis **Z**, r=2.07 mm, len=3.3 mm, centre=(0.0, 0.1, 2.9), support=99
    - axis **X**, r=4.56 mm, len=12.2 mm, centre=(-2.4, -3.5, 0.7), support=90
    - axis **X**, r=4.35 mm, len=12.6 mm, centre=(-2.5, 2.0, 4.1), support=87
    - axis **Y**, r=3.67 mm, len=14.7 mm, centre=(1.7, -1.7, 1.7), support=85

### El.stl

- triangles: **1012**
- bbox size (X,Y,Z): **50.00 x 48.28 x 3.00 mm**
- bbox min / max: (-25.00, -24.14, 0.00) / (25.00, 24.14, 3.00)
- centroid: (2.72, -0.46, 1.50) mm
- solid volume: **6.40 cm^3**   surface area: 49.56 cm^2
- principal extents (PCA, long->short): 49.7, 51.3, 3.0 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **Y**, r=1.54 mm, len=23.2 mm, centre=(22.0, 0.3, 1.0), support=74
    - axis **Z**, r=27.95 mm, len=1.0 mm, centre=(-2.1, 1.9, 1.5), support=70
    - axis **Z**, r=27.79 mm, len=1.0 mm, centre=(-2.4, -2.0, 1.5), support=68
    - axis **X**, r=1.69 mm, len=33.7 mm, centre=(-5.7, 20.4, 2.0), support=61
    - axis **Y**, r=1.49 mm, len=44.6 mm, centre=(10.6, -2.3, 2.0), support=60
    - axis **Z**, r=1.59 mm, len=1.0 mm, centre=(7.9, -19.3, 1.5), support=60
    - axis **Z**, r=1.59 mm, len=1.0 mm, centre=(19.3, 7.9, 1.5), support=60
    - axis **Z**, r=1.59 mm, len=1.0 mm, centre=(8.0, 19.3, 1.5), support=60

### El_Ust.stl

- triangles: **1472**
- bbox size (X,Y,Z): **50.00 x 48.28 x 15.00 mm**
- bbox min / max: (-25.00, -24.14, 0.00) / (25.00, 24.14, 15.00)
- centroid: (-0.10, 0.57, 4.72) mm
- solid volume: **8.45 cm^3**   surface area: 57.79 cm^2
- principal extents (PCA, long->short): 55.5, 55.6, 17.8 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **Y**, r=22.39 mm, len=46.0 mm, centre=(6.0, 1.4, 22.2), support=133
    - axis **Y**, r=4.49 mm, len=46.9 mm, centre=(-16.1, 2.5, 11.0), support=117
    - axis **Y**, r=4.48 mm, len=43.1 mm, centre=(-15.8, 2.7, 6.3), support=114
    - axis **X**, r=31.33 mm, len=48.4 mm, centre=(8.3, -9.8, 31.0), support=92
    - axis **X**, r=58.49 mm, len=47.0 mm, centre=(3.6, 13.5, 61.5), support=88
    - axis **X**, r=48.45 mm, len=41.1 mm, centre=(1.7, 22.0, 48.3), support=86
    - axis **X**, r=10.29 mm, len=42.0 mm, centre=(0.6, -17.4, 12.9), support=78
    - axis **Y**, r=1.54 mm, len=23.2 mm, centre=(22.0, 0.3, 2.0), support=74

### Jack_Cover.stl

- triangles: **978**
- bbox size (X,Y,Z): **25.00 x 16.62 x 12.00 mm**
- bbox min / max: (0.00, -6.91, -0.00) / (25.00, 9.71, 12.00)
- centroid: (8.57, 3.65, 4.12) mm
- solid volume: **1.98 cm^3**   surface area: 18.78 cm^2
- principal extents (PCA, long->short): 28.0, 16.3, 20.8 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **X**, r=5.14 mm, len=19.3 mm, centre=(9.6, 0.0, 7.3), support=196
    - axis **Z**, r=1.39 mm, len=9.0 mm, centre=(7.2, 5.3, 4.5), support=157
    - axis **Y**, r=6.09 mm, len=14.9 mm, centre=(5.4, 4.5, 6.3), support=90
    - axis **X**, r=11.42 mm, len=14.3 mm, centre=(8.0, 5.9, 11.3), support=88
    - axis **Y**, r=1.55 mm, len=1.5 mm, centre=(7.0, 4.3, 5.2), support=85
    - axis **X**, r=4.75 mm, len=11.2 mm, centre=(7.0, 6.5, 7.4), support=74
    - axis **Z**, r=12.27 mm, len=8.7 mm, centre=(12.7, -5.0, 5.9), support=70
    - axis **Z**, r=1.49 mm, len=0.8 mm, centre=(11.5, -0.0, 1.1), support=64

### Mil_1.stl

- triangles: **560**
- bbox size (X,Y,Z): **9.80 x 9.78 x 39.75 mm**
- bbox min / max: (-4.90, -4.89, 0.00) / (4.90, 4.89, 39.75)
- centroid: (-0.00, -0.00, 28.99) mm
- solid volume: **2.81 cm^3**   surface area: 13.13 cm^2
- principal extents (PCA, long->short): 39.8, 9.8, 9.8 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **Z**, r=4.46 mm, len=26.3 mm, centre=(-0.1, 0.0, 25.6), support=192
    - axis **Y**, r=11.73 mm, len=8.5 mm, centre=(-0.3, -0.0, 21.4), support=118
    - axis **X**, r=11.73 mm, len=8.4 mm, centre=(-0.0, 0.3, 21.4), support=116
    - axis **Y**, r=4.03 mm, len=6.7 mm, centre=(0.4, -0.0, 36.0), support=66
    - axis **X**, r=5.25 mm, len=2.3 mm, centre=(-0.0, -0.7, 34.9), support=43

### Mil_2.stl

- triangles: **236**
- bbox size (X,Y,Z): **6.80 x 6.78 x 28.00 mm**
- bbox min / max: (-3.40, -3.39, 0.00) / (3.40, 3.39, 28.00)
- centroid: (0.00, 0.00, 17.92) mm
- solid volume: **1.00 cm^3**   surface area: 6.56 cm^2
- principal extents (PCA, long->short): 28.0, 6.8, 6.8 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **Z**, r=3.39 mm, len=9.0 mm, centre=(-0.0, 0.1, 13.5), support=84
    - axis **Y**, r=3.74 mm, len=4.8 mm, centre=(0.4, -0.0, 24.5), support=50
    - axis **X**, r=4.22 mm, len=1.9 mm, centre=(-0.0, -0.1, 24.2), support=49

### Mil_3.stl

- triangles: **554**
- bbox size (X,Y,Z): **17.00 x 17.00 x 71.80 mm**
- bbox min / max: (-8.50, -8.50, 0.00) / (8.50, 8.50, 71.80)
- centroid: (0.45, -0.09, 50.18) mm
- solid volume: **14.81 cm^3**   surface area: 44.24 cm^2
- principal extents (PCA, long->short): 72.2, 17.2, 18.3 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **Z**, r=8.49 mm, len=47.3 mm, centre=(-0.0, -0.1, 31.7), support=136
    - axis **Y**, r=7.71 mm, len=14.9 mm, centre=(0.6, -0.0, 64.6), support=71
    - axis **Z**, r=1.99 mm, len=6.4 mm, centre=(0.0, 0.1, 61.4), support=64
    - axis **X**, r=11.46 mm, len=5.4 mm, centre=(-0.0, 0.3, 63.3), support=60
    - axis **Y**, r=11.39 mm, len=5.6 mm, centre=(0.3, -0.0, 63.3), support=46
    - axis **X**, r=24.72 mm, len=9.1 mm, centre=(1.8, -6.1, 47.3), support=44
    - axis **X**, r=7.58 mm, len=7.8 mm, centre=(0.5, 5.9, 64.6), support=42
    - axis **Y**, r=16.45 mm, len=16.9 mm, centre=(-6.5, -0.3, 15.6), support=41

### Mil_Disli.stl

- triangles: **2260**
- bbox size (X,Y,Z): **40.26 x 40.13 x 10.00 mm**
- bbox min / max: (-20.13, -19.97, -0.00) / (20.13, 20.16, 10.00)
- centroid: (-0.03, -0.80, 5.00) mm
- solid volume: **8.43 cm^3**   surface area: 44.06 cm^2
- principal extents (PCA, long->short): 40.3, 40.2, 10.0 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **Y**, r=5.29 mm, len=36.3 mm, centre=(-12.1, 0.1, 5.4), support=209
    - axis **Z**, r=15.88 mm, len=3.3 mm, centre=(-0.1, -0.1, 5.0), support=190
    - axis **Y**, r=5.76 mm, len=32.9 mm, centre=(12.9, 1.1, 4.6), support=176
    - axis **X**, r=34.36 mm, len=32.9 mm, centre=(0.0, 16.0, -24.1), support=167
    - axis **X**, r=34.45 mm, len=32.9 mm, centre=(0.0, 16.1, 34.2), support=167
    - axis **X**, r=7.03 mm, len=35.8 mm, centre=(-0.2, -15.8, 3.3), support=136
    - axis **X**, r=10.27 mm, len=35.5 mm, centre=(-0.1, -6.9, 9.9), support=134
    - axis **Y**, r=7.14 mm, len=36.5 mm, centre=(8.9, 1.4, 6.7), support=134

### On_Kol.stl

- triangles: **6240**
- bbox size (X,Y,Z): **172.50 x 56.00 x 40.00 mm**
- bbox min / max: (-152.50, -28.00, -20.00) / (20.00, 28.00, 20.00)
- centroid: (-45.84, 11.15, 0.01) mm
- solid volume: **107.38 cm^3**   surface area: 246.87 cm^2
- principal extents (PCA, long->short): 174.3, 60.5, 40.2 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **Z**, r=52.43 mm, len=23.3 mm, centre=(-8.1, -20.1, -0.2), support=466
    - axis **Y**, r=19.97 mm, len=47.2 mm, centre=(-0.2, 8.1, -0.0), support=317
    - axis **X**, r=10.91 mm, len=157.9 mm, centre=(-57.8, 17.8, 2.1), support=290
    - axis **Y**, r=3.50 mm, len=17.6 mm, centre=(-140.0, -0.7, 0.1), support=249
    - axis **X**, r=4.18 mm, len=55.9 mm, centre=(-22.4, 24.4, -4.5), support=245
    - axis **Z**, r=7.89 mm, len=20.4 mm, centre=(11.8, 20.5, 0.5), support=221
    - axis **Y**, r=4.13 mm, len=32.1 mm, centre=(-34.4, 25.5, 5.0), support=191
    - axis **Y**, r=4.11 mm, len=23.9 mm, centre=(-34.6, 25.7, -5.2), support=189

### Parmak X 2.stl

- triangles: **716**
- bbox size (X,Y,Z): **29.99 x 10.00 x 12.00 mm**
- bbox min / max: (-4.99, -5.00, -6.00) / (25.00, 5.00, 6.00)
- centroid: (13.06, -0.05, -0.00) mm
- solid volume: **2.23 cm^3**   surface area: 17.11 cm^2
- principal extents (PCA, long->short): 30.0, 12.1, 10.0 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **Z**, r=1.50 mm, len=9.8 mm, centre=(19.9, 0.1, 0.0), support=112
    - axis **X**, r=4.44 mm, len=27.6 mm, centre=(11.5, -0.2, 2.0), support=108
    - axis **X**, r=4.35 mm, len=28.8 mm, centre=(11.5, 1.0, -2.0), support=108
    - axis **Z**, r=4.93 mm, len=9.8 mm, centre=(20.1, 0.0, -0.0), support=102
    - axis **Z**, r=5.39 mm, len=4.0 mm, centre=(0.6, -0.0, 0.0), support=58
    - axis **Z**, r=1.50 mm, len=4.0 mm, centre=(0.0, 0.2, 0.0), support=56
    - axis **X**, r=4.52 mm, len=27.9 mm, centre=(15.1, -2.6, -2.0), support=54
    - axis **Y**, r=6.89 mm, len=7.6 mm, centre=(22.4, 0.0, -3.8), support=52

### Parmak_2 X 2.stl

- triangles: **440**
- bbox size (X,Y,Z): **59.86 x 26.21 x 5.00 mm**
- bbox min / max: (-4.99, -5.00, 0.00) / (54.87, 21.21, 5.00)
- centroid: (12.32, 2.07, 2.50) mm
- solid volume: **3.01 cm^3**   surface area: 20.19 cm^2
- principal extents (PCA, long->short): 63.4, 21.1, 5.0 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **X**, r=3.51 mm, len=32.8 mm, centre=(10.4, -1.8, 1.7), support=75
    - axis **X**, r=3.48 mm, len=28.9 mm, centre=(5.7, 1.3, 3.3), support=63
    - axis **Z**, r=1.60 mm, len=1.7 mm, centre=(15.8, 0.0, 2.5), support=60
    - axis **Z**, r=1.59 mm, len=1.7 mm, centre=(0.1, 0.0, 2.5), support=60
    - axis **Y**, r=3.48 mm, len=8.9 mm, centre=(-1.8, -0.2, 3.3), support=53
    - axis **Z**, r=5.03 mm, len=1.7 mm, centre=(0.1, 0.1, 2.5), support=50

### Parmak_Disli X 2.stl

- triangles: **1616**
- bbox size (X,Y,Z): **33.98 x 17.96 x 12.00 mm**
- bbox min / max: (-8.98, -8.98, -6.00) / (25.00, 8.98, 6.00)
- centroid: (4.58, 0.04, -0.00) mm
- solid volume: **3.32 cm^3**   surface area: 24.98 cm^2
- principal extents (PCA, long->short): 34.0, 12.0, 18.0 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **X**, r=8.44 mm, len=31.7 mm, centre=(3.0, 2.8, 2.0), support=211
    - axis **Y**, r=8.26 mm, len=17.4 mm, centre=(-4.6, -0.1, -2.0), support=188
    - axis **Y**, r=8.25 mm, len=17.4 mm, centre=(-4.5, -0.1, 2.0), support=188
    - axis **X**, r=8.25 mm, len=31.7 mm, centre=(3.6, 2.7, -2.0), support=156
    - axis **X**, r=8.31 mm, len=31.5 mm, centre=(1.6, -4.7, -2.0), support=153
    - axis **X**, r=9.85 mm, len=31.0 mm, centre=(1.2, -5.0, 3.7), support=140
    - axis **Y**, r=8.31 mm, len=17.4 mm, centre=(3.9, 0.1, -2.0), support=122
    - axis **Y**, r=8.33 mm, len=17.4 mm, centre=(2.7, 0.1, 2.0), support=115

### Servo_Cable_Holder.stl

- triangles: **596**
- bbox size (X,Y,Z): **46.50 x 16.00 x 33.50 mm**
- bbox min / max: (-23.25, -8.00, -2.00) / (23.25, 8.00, 31.50)
- centroid: (0.04, 0.00, 16.92) mm
- solid volume: **5.56 cm^3**   surface area: 43.43 cm^2
- principal extents (PCA, long->short): 46.6, 33.5, 16.2 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **Y**, r=14.26 mm, len=13.9 mm, centre=(0.0, -0.0, 20.3), support=72
    - axis **X**, r=5.60 mm, len=44.4 mm, centre=(0.2, 2.7, 22.1), support=48
    - axis **X**, r=6.65 mm, len=38.8 mm, centre=(2.3, 1.6, 27.0), support=45
    - axis **Y**, r=22.39 mm, len=13.9 mm, centre=(2.9, -0.0, 9.9), support=44
    - axis **X**, r=5.47 mm, len=45.0 mm, centre=(0.1, 2.7, -1.7), support=42

### Servo_Disli.stl

- triangles: **2692**
- bbox size (X,Y,Z): **40.26 x 40.13 x 10.00 mm**
- bbox min / max: (-20.13, -19.97, -0.00) / (20.13, 20.16, 10.00)
- centroid: (-0.08, -0.10, 5.14) mm
- solid volume: **9.01 cm^3**   surface area: 46.94 cm^2
- principal extents (PCA, long->short): 40.3, 40.2, 10.1 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **Y**, r=5.25 mm, len=36.5 mm, centre=(-11.8, 0.6, 4.8), support=222
    - axis **Y**, r=5.38 mm, len=30.0 mm, centre=(14.1, -0.5, 4.8), support=195
    - axis **Z**, r=15.88 mm, len=3.3 mm, centre=(0.1, 0.1, 5.0), support=190
    - axis **X**, r=7.07 mm, len=36.9 mm, centre=(-0.0, 13.3, 3.3), support=172
    - axis **Y**, r=5.21 mm, len=36.8 mm, centre=(7.2, 1.0, 4.8), support=169
    - axis **Y**, r=6.98 mm, len=38.8 mm, centre=(-3.2, -0.7, 6.7), support=150
    - axis **Z**, r=10.49 mm, len=1.1 mm, centre=(-0.2, 0.1, 8.4), support=144
    - axis **X**, r=8.20 mm, len=35.5 mm, centre=(-0.0, -7.3, 2.3), support=141

### Tabla_Alt.stl

- triangles: **3304**
- bbox size (X,Y,Z): **113.70 x 113.70 x 10.00 mm**
- bbox min / max: (-56.85, -56.85, 0.00) / (56.85, 56.85, 10.00)
- centroid: (0.16, -0.07, 3.26) mm
- solid volume: **19.99 cm^3**   surface area: 145.00 cm^2
- principal extents (PCA, long->short): 113.7, 113.7, 10.0 mm
- detected cylinders (candidate shaft/pivot/mount holes):
    - axis **Z**, r=51.81 mm, len=6.7 mm, centre=(0.2, -0.1, 4.7), support=388
    - axis **Z**, r=56.84 mm, len=2.3 mm, centre=(-0.4, -0.1, 6.5), support=336
    - axis **Z**, r=54.84 mm, len=1.0 mm, centre=(-0.2, 0.3, 1.5), support=332
    - axis **Z**, r=15.07 mm, len=1.0 mm, centre=(0.2, -0.0, 1.5), support=136
    - axis **Z**, r=8.41 mm, len=1.0 mm, centre=(0.2, 0.2, 1.5), support=86
    - axis **X**, r=5.35 mm, len=49.2 mm, centre=(-0.0, -51.5, 5.3), support=82
    - axis **X**, r=5.24 mm, len=50.4 mm, centre=(0.4, 50.9, 5.3), support=76
    - axis **Y**, r=2.17 mm, len=105.7 mm, centre=(10.1, -0.3, 2.0), support=75

## Derived link lengths (pivot-to-pivot estimates)

- **Alt_Kol.stl** (L1 shoulder->elbow (lower arm)): ~**199.4 mm** along axis Z (shaft bosses r=18.0@-105 and r=20.1@94)
- **On_Kol.stl** (L2 elbow->wrist (upper arm)): ~**140.7 mm** along axis X (shaft bosses r=10.9@-141 and r=20.0@-0)
- **Bilek.stl** (wrist pivot block): ~**15.1 mm** along axis X (shaft bosses r=18.6@-3 and r=22.9@12)
