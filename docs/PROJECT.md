# Neptune Arm Simulation — Project Definition

## Mission
Build and understand an articulated 2-DOF manipulator through simulation, validate analytical kinematics and joint-space control, add contact-based manipulation, then adapt Talon's actual arm CAD for the Project Neptune/Stonefish ROV. This repository is independent of the parent Neptune repository.

## Current Phase and Evidence
- **M0**: Python/PyBullet environment and repository initialized.
- **M1**: Fixed-base 2R arm URDF, GUI joint sliders and manual FK verification demonstrated.
- **M2**: Analytical FK/IK, Cartesian target control, quintic joint-space trajectories and modular PyBullet components implemented and **validated locally 6/6**; merged to remote `main` at `5a769e412beafb026cb9aa5b01d411af2f1ef181`.
- **M3**: Two-finger gripper, collision/contact and grasp/lift **not started**. Proceed only when user requests.
- **M4**: Autonomous pick-and-place planned, not started.
- **M5**: Talon CAD articulation/mount integration planned.
- **M6**: Stonefish and Neptune ROV simulation integration planned.

## Implemented Layout and Ownership
- `models/2_dof_arm.urdf`: fixed pedestal, two revolute links and provisional masses/inertias.
- `src/common/config.py`: single source for geometry, limits, motor and physics parameters, and URDF path.
- `src/common/simulation.py`: PyBullet client connection, gravity, timestep, ground, stepping and safe disconnect.
- `src/common/robot.py`: URDF loading, pedestal self-collision pairs, measured joint angles, TCP world pose and contact readings.
- `src/kinematics_control/kinematics.py`: pure FK and cosine-law, two-branch IK with joint-limit checks.
- `src/kinematics_control/trajectory.py`: quintic interpolation for individual and two-joint position targets.
- `src/kinematics_control/controller.py`: stateful `ArmController`, nearest valid IK candidate, trajectory clock/duration, PyBullet position commands.
- `src/kinematics_control/demo.py`: interactive GUI with Cartesian target sliders; preferred M2 entry point.
- `src/kinematics_control/validate.py`: deterministic, headless six-check M2 validation (FK/IK round trip; four endpoint reaches; unreachable rejection).
- `src/run_arm.py`: legacy monolithic development script retained; not the canonical M2 entry point.
- No separate `metrics.py` or `manual_demo.py` is needed for the current scope; essential metrics are calculated in `demo.py` and `validate.py`.

## Mathematical and Physical Model
World axes in PyBullet: +X horizontal, +Z up; motion constrained to X–Z plane. Shoulder height 0.65 m, link lengths 0.30/0.25 m. Shoulder/elbow are revolute about (0,-1,0), each with URDF limits ±2.6 rad. Joint and trajectory angles use radians internally. The M2 TCP is the distal end of Link 2, not a gripper frame.

```text
x = 0.30 cos(q1) + 0.25 cos(q1 + q2)
z = 0.65 + 0.30 sin(q1) + 0.25 sin(q1 + q2)
```

IK uses the law of cosines for ±elbow branches and computes the corresponding shoulder angle, rejecting impossible or joint-limit-violating targets. The controller selects the candidate with smallest squared change from the current **commanded** joint position, then sends quintic time-scaled joint targets. PyBullet uses gravity (0,0,-9.81) m/s² and 1/240 s steps. Motor force cap 12 N·m, max motor velocity 2 rad/s; planned peak speed 1.5 rad/s; minimum duration 2 s.

## Validation and Claims
- Manual M1 GUI demonstrated both joints and analytical FK comparison.
- M2 standalone quintic test produced 0°, 9.32°, 45°, 80.68°, 90° at t=0,0.5,1,1.5,2 s.
- Manual M2 GUI target control reached requested positions, and base collision pushback was visually confirmed after enabling pedestal-vs-link collision pairs.
- Local headless `validate.py` reported **6/6 PASS** with max final Cartesian error 0.000005 m, max independent FK error 0.00000007 m, and max reported final joint error 0.001° across four endpoint tests.
- Endpoint verification is not a proof of collision-free paths, real-world dynamics, or future gripper performance.
- M1 automatic suite was intentionally skipped when M1 closed; M2 has its own now-verified headless validation, not a retroactive M1 run.

## Decisions and Constraints
- Do not duplicate working physics/model/control code across M2, M3, and M4; import from shared modules.
- Keep `demo.py` for interactive exploration and `validate.py` for repeatable checks; preserve old `run_arm.py` until deliberate cleanup.
- No source-code changes during this documentation-only checkpoint.
- M3 should extend/manufacture a genuine articulated gripping mechanism and measure physics contacts, avoiding artificial attachment that fakes grasp.
- M4 sequences M2 motion + M3 gripper using known object positions before perception.
- No collision-free planner, obstacle-avoidance solver, real force/torque identification, or acceleration-continuous replanning is implemented.

## Neptune Integration Contracts — Future M5/M6
Parent repository: https://github.com/Project-Lumyn/Project-Neptune.
- `/state/odometry` is navigation state; `/sim/odometry` is truth for evaluation/debugging only.
- Mission executive is sole normal publisher of `/neptune/cmd_body`.
- `neptune_vehicle` owns command-to-actuator translation.
- Neptune vehicle body frame uses +X bow/forward, +Y starboard, +Z **down**. Do not confuse this with PyBullet world +Z **up**.
- Any public topic/message/frame/unit/sign/ownership semantic change requires `docs/INTERFACES.md`, affected tests and cross-module review in Neptune's parent repository.

## Remaining Open Questions
- M3: finger dimensions and actuator coupling, graspable object geometry/masses, friction/contact tuning and force limits.
- M3/M4: grasp success tolerance and failure detection; repeatable evaluation scenarios.
- M5: actual Talon CAD axes, mass properties, end-effector TCP and mounting transforms.
