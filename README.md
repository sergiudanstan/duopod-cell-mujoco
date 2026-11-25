# DuoPod transfer cell — MuJoCo

[![CI](https://github.com/sergiudanstan/duopod-cell-mujoco/actions/workflows/ci.yml/badge.svg)](https://github.com/sergiudanstan/duopod-cell-mujoco/actions/workflows/ci.yml)
[![CodeQL](https://github.com/sergiudanstan/duopod-cell-mujoco/actions/workflows/codeql.yml/badge.svg)](https://github.com/sergiudanstan/duopod-cell-mujoco/actions/workflows/codeql.yml)
[![OpenSSF Scorecard](https://api.scorecard.dev/projects/github.com/sergiudanstan/duopod-cell-mujoco/badge)](https://scorecard.dev/viewer/?uri=github.com/sergiudanstan/duopod-cell-mujoco)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

A physics simulation in [MuJoCo](https://mujoco.org) of a ceiling-mounted 2-axis parallel robot (modelled on the Autonox DuoPod RVS2-1000-3kg). It moves 3 kg cartons from an infeed conveyor to the conveyor beside it.

**Photoreal view:** the recorded physics run plays in the Pro scene, in **MuJoCo physics** mode: https://sergiudanstan.github.io/duopod-cell-pro/

This project is the physics counterpart of the browser twins [duopod-cell](https://sergiudanstan.github.io/duopod-cell/) and [duopod-cell-pro](https://sergiudanstan.github.io/duopod-cell-pro/). It uses the same geometry, cycle logic and interlocks, now with real contacts, friction, suction and closed-chain dynamics.

## What is simulated

**Robot.** A planar closed-chain mechanism: a 5-bar linkage plus two orientation parallelograms, with the loops closed by `<connect>` equalities.
- The masses, centres of mass and inertias of every part come from the vendor STEP.
- The motor inertia is reflected through the gearbox (`armature = J·i²`).

**Drives.** The servos run at 1 kHz with:
- PD on the joint position, plus a velocity reference;
- feed-forward torque from the analytic rigid-body model;
- the torque limit of the recommended motor (Beckhoff AM8053-0N21, Mmax 53.1 Nm), with an assumed gearbox ratio of 1:40.

**Vacuum gripper.** A MuJoCo `adhesion` actuator. Its force comes from the foam pad: 30 kPa on 70 % of 160 × 220 mm.

**Cartons.** Free bodies, 180 × 250 × 120 mm and 3 kg each.
- The conveyors drive them by friction, capped at μN.
- They queue against a pneumatic escapement and stop at a fixed end stop. The escapement has a blade and a **top clamp**: an earlier side clamp pushed the queued carton against the opposite guide, and it then rubbed the guide while being lifted.

**Cell controller.** It runs on sensors: rangefinder photo-eyes at the end stop, the stop zone, the queue head, the blade gate, the clamp and the belt-B drop zone.
- The escapement opens only when the stop zone is empty and the pick column is clear.
- The robot waits until the belt-B drop zone is clear.
- In the smart cycle, the return goes straight to the next carton but pauses above it until the photo-eye confirms the carton.

## Results (`uv run pytest -q -s`)

| Check | Result |
|---|---|
| Analytic IK → MJCF loops and TCP, 600 × 400 grid | < 0.5 mm |
| Loop closure / plate tilt over the run | 0.05 mm / 0.05° |
| TCP tracking | < 3 mm max |
| 12 cartons transferred, upright, between the guides | yes, 19.4 s |
| Cadence | 39.4 /min (the web twin predicts 39.5) |
| Held carton never touches another carton | yes (fails under `--perturb`, as it must) |
| Held carton clears the 220 mm infeed guides when lifted | ≥ 16.8 mm |
| Support (adapter plate, carriers) clears rod pair 1, triangle and upper arms over the whole 1000 × 400 datasheet range | ≥ 33.7 mm (MuJoCo geometry distance) |
| Measured joint torque vs analytic model, free motion | 2.5 % / 1.3 % RMS difference; peaks 172 / 185 Nm on both |
| Motor torque at i = 40 | peak < 85 % Mmax, RMS < M0 |
| Real-time factor | ~3× on an M-series Mac |

**Robot support.** Rod pair 1 pivots on the head at x = ±166.5 mm, just under the mounting face, and rises outward past the head edge. The triangle and the upper arms rise up to ~210 mm above the face. The adapter plate therefore stays inside the rod pivots (x ±165 mm, inner M20 row) and the carriers sit over it (x ±120 mm). The first layout (700 mm plate, carriers at ±300 mm) was hit by the rods by up to 20.6 mm, and the test above catches it.

## Run

```sh
uv sync
uv run pytest -q -s                 # physics checks
uv run pytest -q --perturb          # escapement interlock removed: the collision test must fail
uv run python tools/run_cycle.py    # headless run with a short report
uv run python tools/render.py       # MP4 + PNG in ~/Downloads/duopod_cell_mujoco/
uv run python tools/export_trajectory.py   # physics run -> ../duopod-cell-pro/physics/duopod_physics.json (photoreal replay)
uv run mjpython tools/view.py       # interactive viewer (macOS; use python on Linux / Windows)
```

## Vendor CAD (optional, local only)

`tools/export_meshes.py` converts a local vendor GLB into STL meshes in `_cad/meshes/` (git-ignored). The model then shows the vendor parts on the same bodies. Without them it shows the procedural geometry.

The vendor CAD is not redistributed. Autonox, DuoPod and Beckhoff are trademarks of their owners.

## Development

`uv sync --locked && uv run pytest -q` runs the physics checks (about 15 s, headless, no rendering). CI also runs `uv run pytest -q --perturb -k test_no_carton_collision_while_held` and requires it to fail. See [CONTRIBUTING.md](CONTRIBUTING.md) for branches and releases, [SECURITY.md](SECURITY.md) for reporting vulnerabilities, and [CHANGELOG.md](CHANGELOG.md) for history.

## Limits

The simulation leaves out:
- joint friction and backlash;
- gearbox elasticity and drive current loops;
- cable forces.

The belt drive is a friction model, not a moving surface. The gearbox ratio is an assumption. Results are a sizing and logic check, not a validated machine.
