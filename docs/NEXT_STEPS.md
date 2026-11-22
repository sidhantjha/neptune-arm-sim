# Active Work and Next Steps

## Current Phase
**M2 — Kinematics and target control** (M1 manually demonstrated, feature branch `feat/m1-two-dof-arm` at `35bb555`).

## Exact Next Action
In `src/run_arm.py`, extract a pure analytical `forward_kinematics(q1, q2)` helper that returns TCP (x,z) in meters (world coordinates, +Z up), and explain/test it against the current PyBullet tip position for a few manually chosen configurations.

## Next Five Actions
1. Factor out and understand independent 2R FK; preserve `get_end_effector_position` as physics-side reference.
2. Implement analytical inverse kinematics for desired TCP (x,z), including reachability, both elbow branches, joint limits, and safe handling of numerical limits.
3. Add target-position input and convert valid targets to shoulder/elbow commands without relying on manual angle sliders.
4. Add smooth time-stepped joint trajectories and report target-versus-actual joint error plus Cartesian TCP error.
5. Manually demonstrate several reachable/unreachable targets, record observations in `EXPERIMENTS.md`, then checkpoint/commit M2.

## M2 Acceptance Criteria
- FK explicitly defined and compared against current PyBullet link transform.
- IK recovers valid joint angles for reachable targets and rejects unreachable targets.
- Selected IK branch satisfies joint limits and avoids obvious self-collisions where possible.
- Target commands move the arm with stable bounded actuator commands.
- Measured position and joint tracking errors are reported.
- User manually verifies at least several distinct targets; tests are not claimed unless actually run.

## Later Phase Outlines
### M3 — Gripper and grasp simulation
- Model and actuate a simple two-finger gripper.
- Configure contact shapes, collision/friction, object mass and gripping force.
- Verify open/close, object contact, pinch grasp and lift; distinguish holding physics from artificial attachment.

### M4 — Autonomous pick-and-place
- Sequence approach -> lower -> grasp -> lift -> move -> release.
- Use known object positions initially.
- Add simple success/failure detection, retries/aborts and trial metrics.

### M5/M6
- Inspect Talon STEP geometry and extract articulated model / mount.
- Convert and integrate the manipulator into Stonefish and the Neptune ROV, respecting Neptune's interfaces and body-frame convention.

## Workstream Rules
- User prefers implementing all code themselves using copy-paste snippets, explained function by function.
- Do not generate downloadable scripts or silently commit code implementation.
- User approved assistant updating the four continuity docs after verifying their feature-branch push.
- M1 automated headless tests were skipped by explicit user choice.

## Active Blockers
None confirmed for M2. The code is on a feature branch; no merge into `main` is verified.

## Continuation Instructions
At a new chat: fetch the branch/main GitHub refs; read `docs/PROJECT.md`, `docs/PROGRESS.md`, `docs/EXPERIMENTS.md`, and this file in full; inspect source, HEAD and local status when possible; report last verified accomplishment and exact next action. Never substitute memory for repository evidence. Keep NEXT_STEPS synchronized with PROGRESS after every meaningful checkpoint.
