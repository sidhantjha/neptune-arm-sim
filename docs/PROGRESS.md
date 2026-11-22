# Project Progress

Last continuity checkpoint: 2026-11-21
Canonical implementation branch: `feat/m1-two-dof-arm`
Last verified implementation commit: `35bb5551b2de96f78bf917479e97b40462d8dc81`
Current phase: **M2 — Kinematics and target position control**

## Milestone Status

| Milestone | Status | Evidence |
|---|---|---|
| M0 Repository and environment | Complete | Initial `main` commit `923d50a`; user demonstrated PyBullet DIRECT connection |
| M1 Basic 2-DOF simulation | Manual demo complete | GUI screenshot/output, two joints and sliders, measured FK agreement; implementation pushed at `35bb555` |
| M2 Kinematics and control | Next | Not implemented |
| M3 Gripper simulation | Pending | Not implemented |
| M4 Automated pick-and-place | Pending | Not implemented |
| M5 Talon CAD integration | Pending | Not implemented |
| M6 Stonefish integration | Pending | Not implemented |

## Verified Accomplishments
- GitHub repository `sidhantjha/neptune-arm-sim` created and initialized.
- On Windows, user successfully activated the `neptune-arm` Conda environment and ran PyBullet with `p.DIRECT` (client 0).
- Two-joint URDF and GUI simulation code pushed on `feat/m1-two-dof-arm` at `35bb555`; `main` remains the original initialization baseline.
- User's GUI screenshot showed the pedestal and articulated arm, shoulder/elbow sliders, and a running physics window.
- User's console output reported two revolute joints with axis (0,-1,0), limits [-2.60,2.60] rad, responsive angles, and FK error displayed as `0.000000 m` at multiple positions.
- Example output: shoulder 25°, elbow -60°, X=0.477 m, Z=0.633 m, FK error=0.000000 m; results matched analytical formulas.
- Earlier `p.gui` capitalization bug corrected to `p.GUI`; a later `KeyboardInterrupt` was caused by stopping the continuously running loop, not evidence of a physics failure.
- User chose to skip headless smoke/regression tests for M1; do not mark those as passed.

## Current Workstream
M1's manual functional acceptance has been demonstrated. Begin M2 by refactoring analytical FK into a function and implementing reachable target -> joint-angle inverse kinematics. Continue in small user-authored, copy-paste code stages with explanations.

## Documentation / Code State
- `src/run_arm.py` and `models/2_dof_arm.urdf` live on the feature branch.
- These docs are updated on that same feature branch; merging to `main` is not yet verified.
- Local user's working tree was clean after push in the supplied terminal screenshot; future local state must be re-checked.

## Checkpoint History
### 2026-11-21 — M0/M1 manual checkpoint
- Prototype completed as manually demonstrated.
- M1 automated tests explicitly deferred by user.
- M2 becomes active; next action in `NEXT_STEPS.md`.

## Verification Rules
Never infer that `main` or local state includes feature-branch changes until confirmed. Record actual tests, parameters, output and commit refs; distinguish manual demonstration from automated regression verification.
