# Venturi CFD

A desktop CFD app with a live preview and direct ParaView export, covering both quick
2D scenarios and custom 3D geometry from an uploaded mesh file. ***NOTE: This program is still under development and does not work perfectly. Use it at your own risk***

## What it does

### 2D Flow Scenarios tab
- Solves the 2D incompressible Navier-Stokes equations (vorticity-streamfunction
  formulation, explicit time-stepping, sparse-LU streamfunction solve reused every
  step).
- Three built-in scenarios: **Lid-Driven Cavity**, **Channel Flow**, and **Flow Past
  a Wall-Mounted Obstacle**.
- Live preview: velocity magnitude + streamlines, plus a convergence (residual) plot.
- Fields exported: `velocity`, `velocity_magnitude`, `vorticity`, `streamfunction`,
  `obstacle`.
### 3D Custom Geometry tab
- Upload an **STL / OBJ / PLY / OFF** file; the app centers, normalizes, and
  voxelizes it onto a Cartesian grid via point-in-mesh testing.
- Solves the 3D incompressible Navier-Stokes equations via an explicit projection
  (Chorin) method: uniform inflow, convective outflow, free-slip walls, and the
  geometry enforced as an immersed solid boundary (a virtual wind tunnel).
- **Multi-core**: kernels are JIT-compiled with numba and parallelized across CPU
  cores (adjustable thread count); expect a modest speedup, often best around 4-8
  threads, since this stencil computation tends to be memory-bandwidth-bound.
- Live preview shows a 2D slice through the mid-plane of the domain.
- **Exports as a real OpenFOAM case**: a full polyMesh (points/faces/owner/
  neighbour/boundary), per-timestep `U`/`p` field files, and a `.foam` placeholder
  ParaView opens directly. The geometry's surface becomes a named boundary patch
  (`object`).
- **Cut-cell surface refinement**: boundary cells are clipped against the actual
  uploaded mesh via boolean difference (`manifold3d`), replacing blocky voxel faces
  with real geometry triangles — roughly 14x lower surface deviation in testing,
  with 90%+ coverage on complex meshes.
- **Orientation check**: after upload, a dialog suggests a flow direction from the
  mesh's principal axes, with a 3D preview to accept, change, or flip it.
  Re-openable via **Re-check Orientation...**.
- **Caching**: results are cached on disk keyed on geometry + settings; identical
  re-runs reuse the cached result instantly (25x+ speedup in testing), and changing
  only Re/steps/threads still reuses the voxelization step. **Force re-run** bypasses
  the cache.
- **Viewer**: A program viewer has been added; this is to prevent ParaView throttling for lower-end laptops and computers. As of
  right now, there is a small delay where the program says it is "not responding." After about 2-3 minutes, it should open the viewer
  and show your model. A more streamlined model with Q-Cores is being developed.
  
## Setup

```bash
python -m venv .venv
.venv\Scripts\pip.exe install -r requirements.txt
```
-Another way to set it up is using the installer.
[https://github.com/yoda-3x3/cfd-studio/blob/main/cfd_studio_cpp/installer/output/VenturiCFDSetup.exe](url)

-Also, **REMEMBER TO INSTALL PARAVIEW TO DISPLAY RESULTS AT [paraview.org](url)
## Running it

```bash
.venv\Scripts\python.exe main.py
```

On Windows this can also be wired up to a Desktop shortcut pointing `pythonw.exe` at
`main.py` with this folder as the working directory, so it launches without a console
window.

## First-time ParaView setup

The app auto-detects ParaView under `C:\Program Files\ParaView*`. If it's installed
elsewhere, use **Settings > Locate ParaView...** to browse to `paraview.exe` once —
the path is remembered in `%APPDATA%\CFDParaviewApp\config.json`.

## Notes

- The 2D solver reports velocity, vorticity, and streamfunction — there is no explicit
  pressure field in that formulation (a common simplification for 2D vorticity-based
  solvers). The 3D solver does report pressure.
- The 3D immersed-boundary treatment is a simple binary voxel mask (cells are either
  fully solid or fully fluid) — the OpenFOAM mesh is exact given that voxelization
  (no further approximation once cells are classified), but the voxelization itself
  is a "staircased" approximation of the uploaded surface, not body-fitted; finer
  grids reduce this. The free-slip domain walls use a boundary treatment that carries
  a small, known, localized divergence artifact right at the 12 domain edges/corners
  — excluded from the reported convergence residual since it doesn't reflect the bulk
  flow. Non-watertight uploaded meshes may also give less reliable inside/outside
  classification near seams.
- ParaView's OpenFOAM reader hides the `0` (initial condition) timestep from the
  animation list by default — that's normal reader behavior (toggle "Skip Zero Time"
  in its properties if you want to see it), not a sign anything is missing.
- Grid resolution, Reynolds number, and step count are all adjustable; time steps are
  chosen automatically for numerical stability. Finer 3D grids and more pressure
  iterations improve accuracy at the cost of runtime and memory.
