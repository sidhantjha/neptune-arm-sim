# Neptune Arm Simulation — Project Definition

## Mission
Develop and validate a two-revolute-joint planar robotic manipulator in PyBullet, demonstrate physical pick-and-place, then adapt Talon's actual arm CAD for integration with Project Neptune's Stonefish ROV simulation.

## Overall Goals and Roadmap
- M0 — Repository and Python/PyBullet environment.
- M1 — Generic articulated two-joint arm, manual motor control, forward-kinematics validation.
- M2 — Standalone forward/inverse kinematics, target position commands, smooth motion and joint tracking metrics.
- M3 — Two-finger gripper, contact properties, grasp and lift validation.
- M4 — Automated approach, grasp, lift, transport and release with repeatable success metrics.
- M5 — Inspect actual Talon STEP files, reconstruct articulated links and mounting geometry, validate against the generic arm.
- M6 — Integrate articulated manipulator into Stonefish and Neptune, including underwater loads and vehicle interaction.

## Implemented M1 Prototype (verified from source at `35bb555`)
- Simulator: PyBullet on Windows, Python 3.11 Conda environment `neptune-arm`.
- Source: `src/run_arm.py`; model: `models/2_dof_arm.urdf`.
- World: +X horizontal forward, +Z upward, gravity (0,0,-9.81) m/s²; 240 Hz fixed physics step.
- Fixed pedestal base with shoulder joint at world height 0.65 m.
- Link 1 length 0.30 m, mass 0.4 kg; link 2 length 0.25 m, mass 0.3 kg; initial inertias are provisional.
- Revolute joints `shoulder` (index 0) and `elbow` (index 1), both rotate about local (0,-1,0); URDF limits ±2.6 rad.
- GUI sliders command degrees; Python converts to radians. PyBullet `POSITION_CONTROL` uses target positions, 12 N·m motor effort cap and max commanded velocity 2 rad/s.
- `get_end_effector_position(robot)` obtains Link 2's world frame through `getLinkState(..., computeForwardKinematics=True)`, then composes a local (0.25,0,0) tip offset with `multiplyTransforms`.
- Analytical FK is independently computed from measured joint angles:
  `x = L1*cos(q1) + L2*cos(q1+q2)`
  `z = 0.65 + L1*sin(q1) + L2*sin(q1+q2)`.
- At M1, the end effector is the tip of Link 2 (a chosen TCP); there is no actuated gripper yet.
- GUI was observed running with sliders and measured FK agreement displayed to six decimal places.

## Proposed M2–M4 Architecture
Task/waypoint planner -> reachability and IK solver -> time-parameterized joint targets -> PyBullet joint controllers -> joint / tip / contact feedback. M3 adds gripper actuation and contact sensing; M4 sequences motions and evaluates task success.

## Technical Design Decisions
- Develop generic geometry before importing Talon CAD.
- Keep arm simulation separate from the parent Neptune repository.
- Prefer small understandable Python functions with explicit math, frames, units, and user-verifiable outputs.
- User elected to skip the automated headless smoke test for M1; manual PyBullet GUI tests are the current evidence. Do not imply continuous regression coverage.
- M2 begins with known desired positions, not perception; M3/M4 should validate actual frictional grasp, not teleport/attach an object.
- Keep underlying URDF and kinematics reusable, but Stonefish requires its own model/actuator integration.

## Neptune Integration Contracts (do not silently change)
Canonical Neptune repository: https://github.com/Project-Lumyn/Project-Neptune
- `/state/odometry` is canonical navigation state; `/sim/odometry` is debug/evaluation truth only.
- Mission executive is the sole normal publisher of `/neptune/cmd_body`.
- `neptune_vehicle` owns body-command-to-actuator translation.
- Neptune body convention: +X forward, +Y starboard, +Z down; PyBullet prototype world +Z is upward.
- Any public interface or ownership change requires review and documentation in Neptune's `docs/INTERFACES.md`.

## Known Limitations and Assumptions
- Link geometry, inertia, joint limits and motor force are example parameters; no physical Talon CAD has been inspected yet.
- No IK, trajectories, gripper, perception, underwater effects or integration implemented yet.
- FK match is a geometry check only, not evidence of realistic motor dynamics or grasping.
- No automatic tests were run for M1 by user choice.
