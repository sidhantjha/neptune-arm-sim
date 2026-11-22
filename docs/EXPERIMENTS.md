# Experiments and Engineering Findings

## Scope and Evidence
Results below are from **developer-provided terminal output and observations** unless stated otherwise. The assistant inspected the merged source code but did not rerun the developer's Windows/PyBullet environment. Commit `5a769e4` contains M2; M3 implementation `304fb40` and M4 validator checkpoint `cf80ff7` were fast-forward merged into `main`. Developer-supplied terminal output establishes M4 tests plus M3/M2 regression; no independent assistant PyBullet execution occurred.

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
| EXP-009 | M2 headless six-check validation; later regression rerun | **PASS — 6/6** both times |
| EXP-010 | M3 URDF and prismatic finger motor open/close | PASS — user-observed |
| EXP-011 | M3 static cube pinch contact and force logging | PASS — user-observed |
| EXP-012 | M3 movable 50 g cube grasp/lift/release | PASS — user-observed |
| EXP-013 | M3 deterministic multi-mass trials | **PASS — 6/6** user-observed |
| EXP-014 | M4 gripper TCP transform and automated demo | **PASS — GUI and headless** user-observed |
| EXP-015 | M4 deterministic multi-mass end-to-end task | **PASS — 6/6** user-observed |
| EXP-016 | M2/M3 regression after M4 changes | **PASS — M2 6/6; M3 6/6** user-observed |

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

## EXP-010 — Gripper URDF and motor positions
Date: 2026-11-22; code later merged at `304fb401db97a7a8215ef44c6858a53517b5f60a`.
URDF: `models/2_dof_gripper.urdf` (separate from M2 arm URDF); joint indices/types: 0 shoulder revolute, 1 elbow revolute, 2 fixed palm mount, 3 left-finger prismatic, 4 right-finger prismatic. Finger joint travel 0–0.035 m each, opposite local Y axes. `GripperController` commands symmetrical opening 0.028–0.098 m with max motor force 8 N and max motor velocity 0.06 m/s. Developer ran `python -m src.grasping.gripper`; output: opened 0.0980 m, closed 0.0280 m; **PASS**. Early source error using `<=` and `>=` rejected exact limit endpoints; changed to strict `<` and `>` before passing.

## EXP-011 — Contact/friction against fixed cube
Command: `python -m src.grasping.contact`.
40 mm cube at world (0.645,0,0.65) m, mass 0.0 (static), friction parameter `lateralFriction=1.0` on cube and both fingers. Open state had contacts (0,0). After closing: actual jaw opening **0.0400 m**, **4 left + 4 right contact points**, summed normal force **8.000 N per finger**. Result: **PASS (user-observed)**. Force is a solver-reported static-contact value, not a calibrated tactile measurement or proof of usable dynamic friction force.

## EXP-012 — Movable cube pinch, lift, and gravity release
Command: `python -m src.grasping.lift`. 40 mm cube with mass **0.050 kg**, supported on static platform before lift. Arm used existing M2 `ArmController` and a 9°-shoulder-derived Cartesian lift target; fingers used M3 `GripperController`. After closure: contacts left=2, right=4, measured gap 0.0401 m. Cube center height before=0.6506 m, after=0.7516 m: lift **0.1009 m**. Contacts after lift left=2, right=3. After opening fingers, cube fell **0.1016 m**. Lift and release **PASS**. No rigid attachment / teleport used; supports genuine physics-contact manipulation for this scenario, not general pick-and-place.

## EXP-013 — Repeated multi-mass M3 validation
Command: `python -m src.grasping.validate`; model: same 40 mm cube geometry and support, mass sweep 0.030 / 0.050 / 0.080 kg, **two independent reset simulation runs each**, identical initial conditions. Each checks gripper open, initially clear contacts, positive bilateral squeeze force, cube rise >0.04 m and retained contacts, and drop >0.04 m after release.

| Mass | Trial | Lift (m) | Drop (m) | Result |
|---|---:|---:|---:|---|
| 0.030 kg | 1 | 0.0973 | 0.0975 | PASS |
| 0.030 kg | 2 | 0.0973 | 0.0975 | PASS |
| 0.050 kg | 1 | 0.1009 | 0.1016 | PASS |
| 0.050 kg | 2 | 0.1009 | 0.1016 | PASS |
| 0.080 kg | 1 | 0.1013 | 0.1015 | PASS |
| 0.080 kg | 2 | 0.1013 | 0.1015 | PASS |

Result: **M3 VALIDATION: 6/6 PASSED (user-observed)**.
M2 regression afterward: `python -m src.kinematics_control.validate` → **6/6 PASSED** with previously observed endpoint errors (max 0.000005 m) unchanged.

Interpretation: consistent grasp, lift and release for three tested masses in a deterministic pose/surface setup. Not a statistical success-rate estimate over varied starts, uncertain object pose, lateral motion, grip friction, or obstacles.

## EXP-014 — M4 autonomous pick-and-place demonstration
Date: 2026-11-22. Feature implementation commit `dc363fdfab3a7f88a7de4009e40fd537ff3c239e`, later validator `cf80ff7ab025da0d3256926a3fb84a8ccbb52d01`.
Commands: `python -m src.pick_and_place.demo` and `python -m src.pick_and_place.demo --gui`.
Environment: Windows PowerShell, Conda `neptune-arm`, PyBullet; generic fixed-base two-revolute arm with parallel-jaw gripper, 40 mm movable cube, known source pose `(0.645,0,0.65) m`, destination `(0.49,0,0.60) m`. Source support top at 0.63 m; destination support top at 0.58 m. M4 includes a joint-space target method and a TCP reach correction from 0.25 m (M2 arm tip) to 0.345 m (gripper center); reported analytical-vs-PyBullet gripper TCP error **0.00000004 m**.

Full history: `HOME -> OPEN -> APPROACH -> ALIGN -> GRASP -> LIFT -> TRANSFER -> PLACE -> RELEASE -> RETREAT -> SUCCESS`.
Headless and GUI both reported `M4 RESULT: PASS`; 50 g final cube pose `(0.4696,0.0016,0.6) m`, target `(0.49,0,0.6) m`, Euclidean placement error **0.0204 m**. This is inside the code's 4 cm acceptance tolerance. Object motion uses PyBullet contact/friction, not a hidden rigid attachment or teleport. The user also visually inspected the GUI, but no quantitative trajectory collision/clearance analysis is recorded.

## EXP-015 — M4 repeated six-trial end-to-end validation
Command: `python -m src.pick_and_place.validate`.
Test definition: independent reset PyBullet episodes with the same 40 mm cube geometry and source/destination poses, masses **0.03, 0.05, 0.08 kg**, **two trials each**. Validator checks `run_episode` returns success, exact expected state sequence, finite Euclidean placement error **<0.04 m**. The task itself checks grasp bilateral contacts, >6 cm lift from initial cube height, continued contacts at transfer/place, final support contact, 4 cm XY/Z placement bounds and low object linear velocity.

Developer's final summary: **`M4 VALIDATION: 6/6 PASSED`**. Excerpted trial details: 0.050 kg trial 2 error=0.0204 m PASS, 0.080 kg trials 1 and 2 error=0.0204 m PASS. Other three trial-specific final errors were not included in the shared excerpt, so do not invent them.

Inference boundary: six deterministic runs do **not** establish statistical robustness to uncertain position, varying surface friction, camera errors, obstacles, or object dimensions.

## EXP-016 — Regression after M4
Developer ran `python -m src.grasping.validate` → **M3 6/6 PASS**, and `python -m src.kinematics_control.validate` → **M2 6/6 PASS** after M4 implementation, before commit/push. The M4-specific `validate.py` was added later on GitHub and then run with **6/6 PASS**; no subsequent M2/M3 rerun after the added validator was separately reported, but the validator only adds a new file.

## Next Experiments (M5 / integration; not yet run)
- Inspect actual Talon manipulator CAD and technical documents; verify joint layout, travel, frames, actuator behavior and hardware mass/inertia estimates from the parent Neptune repository.
- Compare actual arm reach, joint geometry and tool pose against the generic fixed-base planar prototype; document incompatibilities, not assumed equivalence.
- Identify Stonefish articulation/collision/actuator mapping and write a minimal real-arm model validation plan.
- Before claiming deployment readiness: test multiple pickup poses, disturbances, force/friction variation, safety limits and real measured state feedback.

## Technical Debt / Known Limitations
- `src/run_arm.py` is a legacy monolithic script; now-independent `src.kinematics_control.demo` is preferred. No deletion was performed.
- User intentionally omitted standalone `metrics.py`; test/demo compute final errors inline.
- Nearest-branch IK heuristic does not perform obstacle/collision-free path search.
- Repeated GUI changes may cause trajectory replanning and derivative discontinuities.
- Existing M3 code loads gripper URDF directly; `src/common/robot.load_robot` assumes two joints and is used for M2 only. M4 must not naively replace the M2 loader with a five-joint model.
- M3 cube generator has fixed position (0.645,0,0.65) and no parameterized placement; M4 reuses this known-pose scene with a fixed destination, not a variable-pose benchmark.
- The original M2 tip TCP is not the M3 gripper pinch center; M4 introduced a 0.345 m tool-reach correction and verified it at a tested pose. No articulated wrist or collision-free planner was added.
