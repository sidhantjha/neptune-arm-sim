# Active Work and Next Steps

## Current Checkpoint
**M0–M4 complete for the tested PyBullet fixed-base planar prototype. Next is M5: Neptune/Talon-specific manipulator engineering.**
Repository: https://github.com/sidhantjha/neptune-arm-sim
Canonical stable branch: `main`.
Last validated M4 implementation/merge commit: `cf80ff7ab025da0d3256926a3fb84a8ccbb52d01`.
User-observed checks: **M4 6/6**, **M3 6/6**, **M2 6/6** PASS. M4 headless and PyBullet GUI both reached SUCCESS; example final error ~0.0204 m.
This file closes the self-contained educational manipulator prototype; **do not imply that Talon hardware or underwater interfaces are implemented in this repository**.

## Last Verified Accomplishment
- Two-revolute-joint planar arm, analytical FK/IK, quintic joint space control and six-check headless M2 suite.
- Parallel-jaw friction/contact gripper, grasp/lift/release, three masses × two trials M3 suite.
- Full known-pose pick-and-place state machine and six-trial M4 suite, using shared M2 controller and M3 grasp logic. The original Link 2 reach remains 0.25 m, the computed gripper-center reach is 0.345 m.
- User demonstrated both headless and GUI complete autonomous sequence, final cube near `(0.4696,0.0016,0.6000) m` vs `(0.49,0,0.60) m`, error ~0.0204 m.
- M4 feature branch `feat/m4-pick-and-place`, implementation `dc363fd`, validator `cf80ff7`; source was safely fast-forward merged into `main`, with docs updated afterward.

## Exact Next Action — M5, in Parent Project
**Fetch/read canonical [Project-Lumyn/Project-Neptune](https://github.com/Project-Lumyn/Project-Neptune) `main` before Neptune implementation.** Review all seven root continuity files (`AGENTS.md`, `PROJECT_STATE.md`, `ROADMAP.md`, `ACTIVE_WORK.md`, `DECISIONS.md`, `EXPERIMENTS.md`, `HANDOFF.md`) plus relevant `docs/PROJECT_HANDOFF.md`, `docs/ARCHITECTURE.md`, `docs/INTERFACES.md`, `docs/TEAM_CONTRACTS.md`, `docs/OWNERSHIP.md`, `docs/PROJECT_HISTORY.md`. Verify HEAD/branch/remote and preserve existing team work. Check whether Talon CAD/arm work is the current active parent workstream before starting implementation.

## Following M5 Steps (planned, not verified)
1. Inventory authoritative Talon arm CAD, kinematic model, actuator datasheets, mechanical mounting envelope, gripper/end-effector geometry, sensor feedback, and development constraints. Identify missing or unprovided assets rather than invent them.
2. Define the real manipulator's DOF, joint axis frames, signs, limits, link transforms, tool center and base-to-vehicle mounting transform. Compare with educational 2R prototype; isolate reusable math/control patterns from unsuitable assumptions.
3. Determine the most relevant next engineering deliverable in the parent Neptune roadmap: arm CAD integration, model validation, motor/controller interfaces or Stonefish articulation; coordinate module ownership before edits.
4. Build and validate a minimal arm model/controller integration on a **feature branch** with quantitative unit/frame checks, joint limit tests and collision/actuator behavior.
5. Only after clear acceptance criteria and parent repo interface review, integrate arm behaviors into a vehicle-level maintenance/interaction demo and checkpoint parent docs with measurements.

## Critical Contracts
- Parent Neptune body frame: +X bow, +Y starboard, +Z **down**; PyBullet prototype world +Z **up**. Transform frames explicitly.
- `/state/odometry` is canonical navigation state; `/sim/odometry` is simulation truth for evaluation/debugging only. Do not use simulation truth for production autonomy.
- Mission executive is sole normal publisher of `/neptune/cmd_body`; `neptune_vehicle` owns body-command → actuator translation.
- GUI is read-only for motion control. Public topic, frame, units, sign, message or ownership changes require updating `docs/INTERFACES.md`, affected tests and cross-module review.
- Do not equate M4's known object position (task-level simulated ground truth in a learning prototype) with Neptune's production navigation or perception input.

## Prototype Limitations
- Fixed-base two-revolute planar arm, not true Talon mechanism; no independent wrist pose, no collision-free general planner or obstacle avoidance.
- Known fixed object position and fixed destination, hard-coded waypoints and prefer negative elbow IK branch; no camera perception, arbitrary scene layout, disturbance sweep or actual underwater hydrodynamics.
- M4 six-trial results are repeat runs over three cube masses, not a confidence interval or field proof. Example placement error ~2.04 cm; source of systematic offset unresolved.
- Real actuator dynamics, mechanical stops, loads, calibration, saturation and force sensing not established by this prototype.

## Quick Local Sync
```powershell
git switch main
git pull --ff-only origin main
git log -1 --oneline
python -m src.pick_and_place.validate
python -m src.grasping.validate
python -m src.kinematics_control.validate
```

## Continuity
This repository records completed prototype work. For active Neptune engineering, treat the parent GitHub repository's continuity files as authoritative. Avoid copying generic prototype assumptions into Talon without validation. Keep any substantive Neptune changes on a feature branch; no force pushes or overwrite of teammate work. Distinguish verified developer output from source inspection and future plans.
