# Neptune Arm Simulation — Project Definition

## Mission
Develop and validate a 2-DOF robotic manipulator in simulation,
culminating in integration with Project Neptune's Talon ROV.

## Overall Goals
1. Understand and implement robotic arm kinematics.
2. Simulate a two-revolute-joint planar manipulator.
3. Implement forward and inverse kinematics.
4. Control joints and execute trajectories.
5. Simulate a two-finger gripper.
6. Demonstrate autonomous pick-and-place.
7. Inspect and model Talon's actual arm CAD.
8. Integrate the arm into Stonefish and Neptune.

## Initial Scope
- Simulator: PyBullet
- Language: Python
- Two revolute positioning joints
- Separate gripper actuator
- Fixed base
- Simple rigid objects
- Known object locations
- No perception or underwater dynamics initially

## Proposed Architecture
Task Planner
    -> Inverse Kinematics
    -> Trajectory Generator
    -> Joint Controller
    -> PyBullet Physics
    -> Joint/Contact Feedback

## Neptune Integration
Canonical Neptune repository:
https://github.com/Project-Lumyn/Project-Neptune

Future integration:
- Replace generic geometry with Talon arm geometry.
- Build Stonefish-compatible articulated model.
- Mount arm onto the ROV.
- Add separate arm control interfaces.
- Coordinate manipulation with mission executive.

Neptune contract constraints:
- /state/odometry is canonical vehicle navigation state.
- /sim/odometry is simulation truth only.
- Mission executive owns normal /neptune/cmd_body publication.
- neptune_vehicle owns body-command-to-actuator translation.
- Body frame: +X forward, +Y starboard, +Z down.
- Arm integration must not silently change vehicle interfaces.

## Design Decisions
- Begin with generic geometry instead of actual CAD.
- Validate mathematics and control before realistic geometry.
- Use PyBullet for initial development.
- Use Stonefish for underwater integration.
- Keep this repository independent from Neptune initially.

## Assumptions
All dimensions, masses, joint limits, and motor parameters
for the generic arm are provisional.

## Non-Goals for Initial Milestones
- 6-DOF manipulation
- Computer vision
- Advanced grasp planning
- Physical motor control
- Full underwater dynamics
