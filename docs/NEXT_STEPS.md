# Active Work and Next Steps

## Phase
**M3 complete; M4 — autonomous pick-and-place with known object pose — ready to begin.**
Repository: https://github.com/sidhantjha/neptune-arm-sim
Canonical stable branch: `main`.
Last M3 implementation/merge checkpoint: `304fb401db97a7a8215ef44c6858a53517b5f60a`.
M3 six-trial validation: **6/6 PASS**; M2 regression: **6/6 PASS** (developer-provided runs).
The M4 branch should be `feat/m4-pick-and-place`; no M4 implementation has been written or tested at this checkpoint.

## M4 Goal
Given **known simulated object position** and a **fixed target placement position**, automatically perform `HOME → OPEN → APPROACH → ALIGN/LOWER → CLOSE → VERIFY_GRASP → LIFT → TRANSFER → LOWER/PLACE → OPEN/RELEASE → RETREAT → SUCCESS`, reusing existing M2 `ArmController` and M3 `GripperController`. No camera, object-detection inference, teleoperation, or artificially attached object is required for M4 prototype. Use simulated cube pose (e.g. PyBullet `getBasePositionAndOrientation`) as a **task-level provided input**, explicitly marked ground truth; do **not** route simulator truth into Neptune vehicle navigation/control loops.

## Exact Next Action
**Before new task code**, explain and verify the gripper's **actual pinch-center tool frame** relative to Link 2 and the existing M2 FK/IK TCP, then choose a reachable source cube and destination/support layout compatible with the arm's two planar DOFs. The M2 FK tip uses local Link 2 x=0.25 m. The present gripper jaw center is approximately local Link 2 x=0.27+0.02+0.055=0.345 m, so blindly commanding the M2 tip to cube center would miss by ~0.095 m at horizontal orientation. Need account for orientation q1+q2 and jaw pose in world frame. Confirm with PyBullet state, not only nominal values.

## Following Implementation Steps
1. Create `src/pick_and_place/` with `__init__.py` and small, reusable world/geometry or task configuration (known pickup pose, target pose, support, safe approach/lift clearances). Reuse `src/grasping/contact.py` and `lift.py`; modify cube spawn function only if needed, preserving defaults and M3 validation.
2. Implement a compact `task.py` state machine that owns sequencing but delegates joint commands to existing M2 arm controller and M3 finger controller. Keep state and timeout thresholds explicit; no duplicate motor or IK implementations.
3. Implement one full task demonstration (`demo.py` or a minimal `task.py` entrypoint, avoid redundant files) with known cube pose. Validate approach alignment, bilateral contacts, lift, transport, safe placement, fingers opening and retreat.
4. Add `validate.py` with repeatable full-task trials and logged metrics: success/failure by state, contact persistence, peak object slip, final placement XY/Z error, task steps/simulation time and recovery/abort behavior.
5. Rerun M3 6/6 plus M2 6/6 regression after changes; checkpoint test configurations/results, preserve `main` until passing; push feature branch, fast-forward merge safely, update README and four continuity docs.

## M4 Initial Acceptance Criteria (proposed; adjust before validation)
- Starting object pose and destination are explicitly provided or read from simulation only at the **task input** boundary.
- Arm's commanded gripper/pinch center reaches intended pickup and place waypoints to specified tolerances; any unreachable target fails safely.
- Object remains dynamically simulated, held by collision/friction (no teleport, rigid constraint, or forced attachment).
- All required task states occur in order; transitions depend on measured state/contacts/timeouts rather than assuming a command instantly completes.
- Successful pickup lifts cube clear of source support; transfer does not lose contact; object is released and rests stably at destination.
- Test reports final object placement error and grasp/transfer success for each trial; M2/M3 regression remains green.

## Existing Code Reuse
- `models/2_dof_gripper.urdf`: arm (revolute indices 0/1), fixed gripper mount (2), prismatic fingers (3/4).
- `src/common/simulation.py`: physics and time stepping; `src/common/config.py`: link lengths/time step.
- `src/kinematics_control/{kinematics,trajectory,controller}.py`: analytical FK/IK, quintic joint commands, arm controller.
- `src/grasping/gripper.py`: symmetric opening commands; `src/grasping/contact.py`: cube spawning, friction, bilateral contacts; `src/grasping/lift.py`: support and cube-pose inspection.
- Keep `models/2_dof_arm.urdf` and `src.common.robot.load_robot` for M2's two-joint tests; the M3/M4 gripper URDF is a distinct five-joint asset. Do not treat the shared M2 loader's `num_joints==2` assumption as valid for gripper model.
- `src/run_arm.py` is still historical monolithic code; no need to remove during M4 unless the user explicitly asks.
- No dedicated metrics module needed until task metrics genuinely become reusable.

## Known Constraints and Risks
- The arm only moves in the X–Z plane. Pinch occurs along **local ±Y**; object approach and placement must be geometrically feasible with arm pose and wrist orientation coupled to q1+q2.
- The old FK/IK targets the *tip of Link 2*, not the grasp center. This offset matters for autonomous precise pickup/placement.
- The M2 closest-joint IK solution and joint-space quintic trajectories do **not** guarantee collision avoidance; choose conservative waypoints and monitor contacts during transitions.
- Object mass, friction and motor parameters are generic, not calibrated to Talon real hardware.
- M3 validation is deterministic: single cube size 40 mm, one starting pose, three masses with two repeats each. Extend the scenario set in M4 rather than claiming broad robustness.

## Continuity / Repository Safety
Read README, `docs/PROJECT.md`, `docs/PROGRESS.md`, `docs/EXPERIMENTS.md` and this file before resuming. Check branch, remote main SHA and working tree. Work on `feat/m4-pick-and-place` and keep main stable; never force push or overwrite another contributor. User prefers small code snippets to paste and test one file at a time. GitHub is authoritative; record verified observations separately from proposed designs.
