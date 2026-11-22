# Experiments and Engineering Findings

## Scope and Evidence
Results below are from **developer-provided terminal output and observations** unless stated otherwise. The assistant inspected the merged source code but did not rerun the developer's Windows/PyBullet environment. Commit `5a769e4` contains the completed M2 prototype on `main`.

## Index

| ID | Test / experiment | Result |
|---|---|---|
| EXP-001 | PyBullet DIRECT environment | PASS — user-observed |
| EXP-002 | M1 GUI joints and analytical FK comparison | PASS — manual |
| EXP-003 | M1-specific headless smoke suite | SKIPPED — by user |
| EXP-004 | Analytical FK/IK two-branch round trip | PASS — user-observed |
| EXP-005 | Standalone quintic trajectory sampling | PASS — user-observed |
| EXP-006 | GUI Cartesian targeting with quintic trajectories | PASS — manual |
| EXP-007 | Pedestal collision pushback | PASS — user visual confirmation; force not recorded |
| EXP-008 | Shared simulation/robot module checks | PASS — user-observed |
| EXP-009 | M2 headless six-check validation | **PASS — 6/6** |

## EXP-001 — Environment
Date: 2026-11-21. Windows PowerShell, Conda `neptune-arm`, Python 3.11; PyBullet build reported Nov 21, 2026.
User ran a DIRECT client smoke test and observed `PyBullet works! 0`. Result: PASS.

## EXP-002 — M1 GUI arm and forward kinematics
Implementation `35bb555`, URDF `models/2_dof_arm.urdf`. Fixed shoulder height 0.65 m, links 0.30/0.25 m, axis (0,-1,0), joints ±2.6 rad, position-control effort cap 12 N·m, timestep 1/240 s. User moved sliders and observed joint angles responding.
At shoulder 25°, elbow -60°: X≈0.477 m, Z≈0.633 m, FK error displayed as 0.000000 m. Other sampled configurations also showed displayed zero FK error. Result: PASS (manual). Early `p.gui` case mismatch was corrected to `p.GUI`. Ctrl+C interrupt of continuous loop was not a physics failure.

## EXP-003 — M1-specific automated tests
Explicitly skipped at M1 by user preference. This does **not** imply that later M2 validation was skipped; EXP-009 was successfully run.

## EXP-004 — Analytical IK
M2 prototype: target generated from FK of (25°,-60°) at (0.4767,0.6334) m. User reported IK solutions (-28.99°,60°) and (25°,-60°), each with printed 0.00000000 m round-trip error. Result: PASS. No full workspace sweep.

## EXP-005 — Quintic trajectory generator
User ran `python -m src.kinematics_control.trajectory` and observed one-joint command sampled from 0° to 90° across 2 s:
- 0.0 s → 0.00°
- 0.5 s → 9.32°
- 1.0 s → 45.00°
- 1.5 s → 80.68°
- 2.0 s → 90.00°
Result: PASS. Interpolation is in joint space, not guaranteed straight-line TCP motion.

## EXP-006 — M2 GUI trajectory control
User ran interactive trajectory control and moved target sliders. A 2-second trajectory settled at target (0.400,0.992) m with displayed 0.0000 m target error and approximately 0.00° joint error. Frequent 'New target' lines while dragging sliders demonstrated repeated replanning; after slider release the path completed. Result: PASS for manually observed endpoint tracking. No guarantee of jerk/velocity continuity on midflight replans.

## EXP-007 — Pedestal collisions
Initial attempted collision filters referenced `robot` and then `urdf_path` before creation, causing `UnboundLocalError`; corrected by resolving URDF path, loading the robot, then configuring base↔Link 1 and base↔Link 2 collision pairs. User later reported visible physical pushback. Initial console output of `Base/forearm contacts: 0` did not prove numerical base force; only forearm was counted, and printed snapshots may miss brief contacts. Result: **manual visual pushback confirmed; no measured force/contact regression acceptance recorded**.

## EXP-008 — Shared simulation and robot module verification
Developer ran:
- `python -m src.common.simulation` → Connected: 1; Bodies loaded: 1; Advanced 240 physics steps successfully; Simulation closed successfully.
- Import-and-load check in DIRECT mode → joint angles (0.0,0.0), tip (0.550000011920929,0.0,0.6499999761581421) m.
Result: PASS (user-observed).

## EXP-009 — M2 full-stack headless endpoint validation
Date: 2026-11-21. Implementation merged to `main` at `5a769e412beafb026cb9aa5b01d411af2f1ef181`.
Command: `python -m src.kinematics_control.validate`.
Configuration: same generic arm; PyBullet DIRECT, gravity (0,0,-9.81) m/s², 1/240 s physics step; `ArmController` with position-control max effort 12 N·m, max velocity 2 rad/s, planned speed limit 1.5 rad/s; minimum trajectory 2 s. For reach trials, run planned duration + 0.75 s settling.
Acceptance thresholds: target error ≤0.01 m, joint final error ≤2°, independent FK error ≤0.0001 m.

| Check | Result | Final target error (m) | Final FK error (m) | Max final joint error |
|---|---|---:|---:|---:|
| FK/IK round-trip (25°,-60°) | PASS | — | Below 0.0001 | — |
| Initial reach (0.350,0.850) | PASS | 0.000001 | 0.00000007 | 0.000° |
| Horizontal reach (0.450,0.650) | PASS | 0.000005 | 0.00000005 | 0.001° |
| Raised reach (0.250,1.000) | PASS | 0.000001 | 0.00000005 | 0.000° |
| Final reach (0.400,0.900) | PASS | 0.000003 | 0.00000005 | 0.001° |
| Reject unreachable (0.500,1.100) | PASS | — | — | — |

Reported final summary: `M2 VALIDATION: 6/6 PASSED` and `M2 motion system validated successfully!`.
Conclusion: M2 prototype endpoint tracking, FK consistency, and unreachable-target handling pass the defined tests. **This is not evidence of collision-free trajectories, obstacle avoidance, grasp stability, or field performance.**

## Next Experiments (not yet run)
- M3: finger open/close articulation; verify both fingers physically move and limits are observed.
- M3: friction/contact test with small object; contact forces during pinch and lift; release verification.
- M3: multiple object masses/sizes and repeated grasp trials with explicit success criteria.
- M4: autonomous full pick-and-place trials with known object coordinates, failure handling, success rate.

## Technical Debt / Known Limitations
- `src/run_arm.py` is a legacy monolithic script; now-independent `src.kinematics_control.demo` is preferred. No deletion was performed.
- User intentionally omitted standalone `metrics.py`; test/demo compute final errors inline.
- Nearest-branch IK heuristic does not perform obstacle/collision-free path search.
- Repeated GUI changes may cause trajectory replanning and derivative discontinuities.
