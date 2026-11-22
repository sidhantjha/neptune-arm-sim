# Neptune Arm Simulation — Project Definition

## Mission
Build and understand an articulated 2-DOF manipulator through simulation, validate analytical kinematics and joint-space control, add contact-based manipulation, then adapt Talon's actual arm CAD for the Project Neptune/Stonefish ROV. This repository is independent of the parent Neptune repository.

## Current Phase and Evidence
- **M0**: Python/PyBullet environment and repository initialized.
- **M1**: Fixed-base 2R arm URDF, GUI joint sliders and manual FK verification demonstrated.
- **M2**: Analytical FK/IK, Cartesian target control, quintic joint-space trajectories and modular PyBullet components implemented and **validated locally 6/6**; merged to remote `main` at `5a769e412beafb026cb9aa5b01d411af2f1ef181`.
- **M3**: Parallel-jaw gripper with two prismatic fingers, contact/friction, movable-cube pinch, physical lift and release. **Implemented and user-validated 6/6** at 30/50/80 g; M2 regression 6/6; merged to `main` at `304fb401db97a7a8215ef44c6858a53517b5f60a`.
- **M4**: Autonomous pick-and-place using known object pose is the next active workstream; **not implemented**.
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
- `models/2_dof_gripper.urdf`: original two joints plus a fixed palm mount and left/right prismatic finger joints (indices 3/4). Kept separate from M2-only URDF.
- `src/grasping/gripper.py`: `GripperController` commands symmetric 28–98 mm opening via PyBullet position control; reads measured opening.
- `src/grasping/contact.py`: static/movable 40 mm cube factory, lateral friction configuration, finger contact-point readings.
- `src/grasping/lift.py`: 50 g cube support, grasp, approximately 10 cm vertical lift and gravity-based release demonstration.
- `src/grasping/validate.py`: six repeatable trials, 30/50/80 g twice each, full grasp/lift/release outcome checks.
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
- Endpoint verification is not a proof of collision-free paths or real-world dynamics.
- M3 tests: finger open/close PASS; fixed 40 mm cube squeeze PASS (four contact points per finger, summed normal force 8 N each); movable 50 g cube lift 0.1009 m and release drop 0.1016 m PASS; M3 six-trial headless validator 6/6 PASS at 30/50/80 g; M2 regression 6/6 PASS. These are user-reported local runs, not independently rerun by the assistant. Trials are repeated under identical initial conditions, not randomized disturbance tests.
- M3 model mounting: gripper palm joint at local Link 2 x=0.27 m; finger centers at local palm x ≈0.075 m, so the current grasp center is **not the same point as the M2 arm TCP** at Link 2 x=0.25 m. M4 must explicitly transform between the grasp center, the Link 2 frame, and world targets and account for forearm orientation.
- M3 finger opening range: 0.028–0.098 m, with two prismatic finger joints; finger motor limit set to 8 N and speed 0.06 m/s, friction coefficient parameter 1.0 for finger and cube. All settings provisional.
- M1 automatic suite was intentionally skipped when M1 closed; M2 has its own now-verified headless validation, not a retroactive M1 run.

## Decisions and Constraints
- Do not duplicate working physics/model/control code across M2, M3, and M4; import from shared modules.
- Keep `demo.py` for interactive exploration and `validate.py` for repeatable checks; preserve old `run_arm.py` until deliberate cleanup.
- No source-code changes during this documentation-only checkpoint.
- M3 does use articulated prismatic fingers and measured frictional contacts, with no fake attachment. The model now exists in a separate URDF to preserve M2 regression behavior.
- M4 will compose M2 arm motion with M3 gripper through a state-machine task executive (home, approach, align, close, verify, lift, transfer, release, retreat), using a **known simulated object pose** for initial planning rather than perception. Simulation ground truth is restricted to this M4 task prototype, not Neptune navigation/control.
- No collision-free planner, obstacle-avoidance solver, real force/torque identification, or acceleration-continuous replanning is implemented.

## Neptune Integration Contracts — Future M5/M6
Parent repository: https://github.com/Project-Lumyn/Project-Neptune.
- `/state/odometry` is navigation state; `/sim/odometry` is truth for evaluation/debugging only.
- Mission executive is sole normal publisher of `/neptune/cmd_body`.
- `neptune_vehicle` owns command-to-actuator translation.
- Neptune vehicle body frame uses +X bow/forward, +Y starboard, +Z **down**. Do not confuse this with PyBullet world +Z **up**.
- Any public topic/message/frame/unit/sign/ownership semantic change requires `docs/INTERFACES.md`, affected tests and cross-module review in Neptune's parent repository.

## Remaining Open Questions
- M4: define grasp-center world transform, approach clearance and orientation constraints for the 2-DOF X–Z arm; select reachable pickup and placement supports.
- M4: define state transitions, settle/contact thresholds, timeouts, post-lift object stability and place-position tolerance.
- M4: known simulated object pose can be read from PyBullet initially; later perception must replace that input without silently changing navigation state contracts.
- M5: actual Talon CAD axes, masses, inertia, grasp TCP and mounting transforms.
