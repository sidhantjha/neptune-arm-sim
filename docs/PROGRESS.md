# Project Progress

Last continuity checkpoint: 2026-11-21 — documentation synchronized after M2 merge
Canonical branch: `main`
Last verified implementation merge: `5a769e412beafb026cb9aa5b01d411af2f1ef181`
Current phase: **M2 complete; M3 queued, awaiting explicit start**

## Milestone Status

| Milestone | Status | Evidence |
|---|---|---|
| M0 — Repository/environment | Complete | Working Windows Conda/PyBullet environment and initial repo |
| M1 — Basic 2-DOF simulation | Manual demo complete | Two joints, position sliders, URDF, FK comparison; M1 code at `35bb555` |
| M2 — Kinematics and control | **Complete for defined M2 prototype scope** | Modular implementation `5a769e4` on `main`; developer's local six-check validation passed |
| M3 — Two-finger gripper and contact grasp | Not started | Next planned phase; user requested pause |
| M4 — Automated pick-and-place | Pending | Depends on M3 |
| M5 — Talon CAD arm integration | Pending | Actual CAD not processed |
| M6 — Stonefish/Neptune integration | Pending | Not started |

## Last Verified Accomplishments
- Developer successfully ran `python -m src.common.simulation`: one ground body loaded, 240 physics steps advanced, clean shutdown.
- Developer successfully ran headless `src.common.robot` import-and-load check: initial joint angles (0,0), tip approximately (0.550000012,0,0.649999976) m.
- Developer previously confirmed M2.1 standalone FK and two valid IK solutions (25°,-60°) and (-28.99°,60°), with displayed zero round-trip error.
- Developer previously confirmed standalone quintic trajectory sampling and a GUI M2 trajectory that settled at Cartesian target (0.400,0.992) m with zero displayed error.
- Developer reported collision pushback as manually visually confirmed after enabling pedestal-vs-moving-link contact filtering; contact force was not quantitatively verified in saved output.
- **Local M2 `python -m src.kinematics_control.validate`: 6/6 PASS**:
  - FK/IK round trip PASS.
  - Initial (0.350,0.850) m: final target error 0.000001 m, FK error 0.00000007 m, max joint error 0.000°.
  - Horizontal (0.450,0.650) m: 0.000005 m, 0.00000005 m, 0.001°.
  - Raised (0.250,1.000) m: 0.000001 m, 0.00000005 m, 0.000°.
  - Final (0.400,0.900) m: 0.000003 m, 0.00000005 m, 0.001°.
  - Unreachable target (0.500,1.100) m: correctly rejected and prior goal retained.
  - Tolerances: target error ≤0.01 m; FK error ≤0.0001 m; joint error ≤2°.
- Local `feat/m2-kinematics` commit `5a769e4` pushed; user fast-forward merged into `main` and pushed; reported clean working tree with `main` tracking `origin/main`. Assistant subsequently confirmed remote `main` contains `5a769e4`.

## Current Source State
- Main M2 entry point: `python -m src.kinematics_control.demo`.
- Headless M2 validator: `python -m src.kinematics_control.validate`.
- Reusable modules: `src/common/{config,simulation,robot}.py`, `src/kinematics_control/{kinematics,trajectory,controller}.py`.
- `src/run_arm.py` is still present, monolithic, and redundant for the new M2 path. Do not describe it as deleted or refactored to a minimal launcher.
- No dedicated `metrics.py`; intentionally skipped by user. Core errors live in demo/validator.
- No gripper code, M3 tests or M4 task logic yet.

## Remaining Limitations
- The six M2 checks cover four endpoint motions, one FK/IK round trip, and one unreachable-target rejection; they do not certify all paths or collision-free planning.
- Joint-space quintic interpolation has endpoint zero planned velocity/acceleration for completed trajectories, but mid-flight target changes can be velocity/acceleration discontinuous.
- Closest-IK-branch heuristic can select unsafe joint-space motions; collision response is not motion planning.
- Dynamics and geometry are generic examples, not calibrated to Talon hardware.

## Checkpoint History
- **2026-11-21 M1:** M0+M1 manual demonstration; M1 automatic smoke test deferred; implementation `35bb555`, docs `e721c38`.
- **2026-11-21 M2 implementation:** modular FK/IK, motion controller, GUI demo, headless validator; user reported 6/6 PASS, committed and merged to main as `5a769e4`.
- **2026-11-21 documentation checkpoint:** README and four canonical arm continuity files updated on `main`; no code implementation changes.

## Continuation Rules
Repository state is authoritative over conversation memory. Confirm remote branch/HEAD and read the four docs before resuming. Distinguish user-observed local results from source code inspection. User writes and tests code one file at a time, prefers simple explanations; assistant can manage approved documentation checkpoints. Do not begin M3 until requested.
