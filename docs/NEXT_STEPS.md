# Active Work and Next Steps

## Current Status
**M2 is complete for the prototype's verified scope. M3 is queued, not yet started.**
Repository: https://github.com/sidhantjha/neptune-arm-sim
Canonical remote branch: `main`.
Last verified M2 implementation/merge commit: `5a769e412beafb026cb9aa5b01d411af2f1ef181`.
The user explicitly asked to finish GitHub documentation **and pause before proceeding to the next phase**.

## Exact Next Action — Only After User Says to Begin M3
Start an M3 gripper-design discussion grounded in the current 2-DOF URDF, and agree on a minimal two-finger gripper model (mount/TCP location, finger joints/actuation, objects, friction and measurable grasp criteria) **before changing source code**. Preserve `main` as stable baseline; use a new feature branch for M3 implementation.

## Following Five Steps for M3
1. Define gripper geometry, mount transform relative to Link 2's TCP, joint axes and ranges, and object test geometry. Decide what "successful grip/lift" means.
2. Create an M3 feature branch and extend or add a URDF with physically modeled moving fingers, contact/collision shapes, and inertias; keep the original arm demo functional.
3. Add reusable gripper control/state/contact helpers, reusing `src/common` simulation and existing M2 arm IK/trajectory controller.
4. Demonstrate opening/closing and object contact, then lift, hold and release a known object; capture contact forces where available. No fake teleport/attachment as the sole grasp mechanism.
5. Run repeatable grasp trials with explicit object mass/friction and pass/fail metrics; record experiments, checkpoint docs, then merge only after verification.

## M3 Acceptance Criteria — Proposed, Not Yet Approved
- Fingers respond to open/close commands within modeled joint limits.
- Grasp depends on physically simulated contacts/friction, not hidden rigid attachment.
- A test object can be contacted, lifted, carried briefly and released.
- Logs include grasp outcomes, example contact forces and failure modes.
- Existing M2 6/6 headless validation still passes after gripper-related changes, or any intended interface change is explicitly documented and tested.

## M4 Outline
Use known object coordinates initially. Compose M2 motion and M3 gripper control in a task state machine: HOME → APPROACH → LOWER → CLOSE → VERIFY → LIFT → TRANSFER → RELEASE → RETREAT. Add timeouts, recovery logic and repeated-trial metrics; no perception required for the first M4 demonstration.

## M5–M6 Outline
M5: inspect real Talon STEP/CAD, identify link/joint frames, realistic mass/inertia, actuator constraints and mount. M6: implement Stonefish articulated arm and Neptune vehicle integration with explicit frame/units/interface contract review.

## Immediate Verification Commands
From the repository root with the `neptune-arm` environment active:
```powershell
git status
git log -3 --oneline
python -m src.kinematics_control.validate
python -m src.kinematics_control.demo
```
The headless M2 validation was last reported as `6/6 PASSED` after `5a769e4`; don't claim a fresh rerun until one occurs.

## Important Constraints and Continuity
- Shared code lives in `src/common/{config,simulation,robot}.py`; math/trajectory/controller in `src/kinematics_control/`. Prefer `python -m src.kinematics_control.demo`.
- `src/run_arm.py` remains as an untouched historical monolithic script; remove only with user's explicit agreement and after verifying no references rely on it.
- `metrics.py` and redundant manual demos were intentionally skipped.
- Generic PyBullet world +Z up vs Neptune vehicle body +Z down. Parent repo interfaces and control ownership are unchanged.
- Joint-space trajectories are not collision-free plans; user manually observed base collision pushback, without quantitative force evidence.
- Do not implement source code or begin M3 until user explicitly asks.
- User prefers teaching-led, one-file-at-a-time copy/paste implementations; assistant may write approved GitHub continuity documentation.
- For a new chat, fetch remote refs; read README, `PROJECT.md`, `PROGRESS.md`, `EXPERIMENTS.md`, `NEXT_STEPS.md` and inspect relevant source before continuing. Repository beats remembered history.
