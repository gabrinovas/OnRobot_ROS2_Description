# Original meshes

The meshes of `meshes/` listed below were simplified on 2026-11-13 so that RViz, robot_state_publisher and MoveIt
load them quickly. This folder keeps the originals, with the same relative paths, and is not installed.
To restore one, copy it back over the file of the same name in `meshes/`.

How they were simplified: VTK 9.1 quadric decimation (vtkQuadricDecimation) after merging duplicate vertices.
The deviation is the largest distance between the original and the simplified surface, measured from every
vertex of each one to the other (the meshes are in millimetres). Limits used: 0.5 mm for visual meshes and
2 mm for collision meshes.

| Mesh | Use | Triangles before | Triangles now | Deviation (mm) | Size before | Size now |
|---|---|---|---|---|---|---|
| `2fg7/visual/Quick_Changer_Robot_side_1.stl` | visual | 43,560 | 5,000 | 0.311 | 2.1 MB | 0.24 MB |
| `2fg7/visual/official_base_onrobot.stl` | visual | 359,084 | 10,000 | 0.329 | 17.1 MB | 0.48 MB |
| `3fg15/visual/base.stl` | visual | 326,729 | 9,999 | 0.295 | 15.6 MB | 0.48 MB |
| `3fg15/visual/left_finger.stl` | visual | 32,402 | 5,000 | 0.190 | 1.5 MB | 0.24 MB |
| `3fg15/visual/middle_finger.stl` | visual | 32,402 | 5,000 | 0.192 | 1.5 MB | 0.24 MB |
| `3fg15/visual/right_finger.stl` | visual | 32,402 | 5,000 | 0.190 | 1.5 MB | 0.24 MB |
| `vgc10/visual/base_vgc.stl` | visual | 436,791 | 19,998 | 0.415 | 20.8 MB | 0.95 MB |
| `vgc10/visual/ventosa_15_mm.stl` | visual | 21,760 | 5,000 | 0.057 | 1.0 MB | 0.24 MB |
| `vgc10/extra_parts/ventosa_30_mm.stl` | visual | 23,118 | 5,000 | 0.156 | 1.1 MB | 0.24 MB |
| `vgc10/extra_parts/ventosa_40_mm.stl` | visual | 23,662 | 5,000 | 0.116 | 1.1 MB | 0.24 MB |
| `2fg7/collision/Quick_Changer_Robot_side_1.stl` | collision | 43,560 | 1,999 | 1.871 | 2.1 MB | 0.10 MB |
| `2fg7/collision/official_base_onrobot.stl` | collision | 359,084 | 4,000 | 1.337 | 17.1 MB | 0.19 MB |
| `2fg7/collision/official_left_finger.stl` | collision | 13,460 | 500 | 0.968 | 0.6 MB | 0.02 MB |
| `2fg7/collision/official_right_finger.stl` | collision | 13,460 | 500 | 0.968 | 0.6 MB | 0.02 MB |
| `3fg15/collision/base.stl` | collision | 326,729 | 1,998 | 1.832 | 15.6 MB | 0.10 MB |
| `3fg15/collision/left_finger.stl` | collision | 32,402 | 999 | 1.273 | 1.5 MB | 0.05 MB |
| `3fg15/collision/middle_finger.stl` | collision | 32,402 | 999 | 1.311 | 1.5 MB | 0.05 MB |
| `3fg15/collision/right_finger.stl` | collision | 32,402 | 500 | 1.855 | 1.5 MB | 0.02 MB |
| `vgc10/collision/base_vgc.stl` | collision | 436,791 | 3,998 | 1.799 | 20.8 MB | 0.19 MB |
| `vgc10/collision/ventosa_15_mm.stl` | collision | 21,760 | 500 | 0.334 | 1.0 MB | 0.02 MB |
