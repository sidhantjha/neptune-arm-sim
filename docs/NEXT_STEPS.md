# Active Work and Next Steps

## Active Milestone
M0 — Repository and Environment Setup

## Exact Next Action
Create the local Git repository, initialize its documentation,
create the GitHub remote, and verify the initial push.

## Next Five Actions
1. Initialize the repository and push the first commit.
2. Create the Python virtual environment.
3. Install and verify PyBullet.
4. Build the initial two-link URDF model.
5. Load the robot and manually control both joints.

## M1 Acceptance Criteria
- Robot model loads without errors.
- Two revolute joints exist.
- Joint limits are configured.
- Both joints respond to commands.
- Link geometry appears correctly.
- Joint angles can be read programmatically.
- Basic tests pass.

## Upcoming Milestones
M2: Forward/inverse kinematics and trajectories.
M3: Gripper and physical contact.
M4: Automated pick-and-place.
M5: Real CAD integration.
M6: Stonefish and Neptune integration.

## Active Blockers
None verified.

## Open Decisions
- Exact initial link dimensions
- Robot mass and inertia estimates
- Initial actuator limits
- Gripper design and control strategy

## Continuation Instructions
At the start of a new chat:
1. Read PROJECT.md.
2. Read PROGRESS.md.
3. Read EXPERIMENTS.md.
4. Read NEXT_STEPS.md.
5. Verify repository HEAD and working-tree status.
6. Report the current phase and next exact action.
7. Continue without repeating completed work.

At each checkpoint, update the relevant files and ensure
NEXT_STEPS.md reflects the next executable task.
