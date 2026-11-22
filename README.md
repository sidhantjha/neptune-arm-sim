# Neptune Arm Simulation

A simulation-first learning and engineering project for a **two-revolute-joint (2-DOF) planar robotic arm with a parallel-jaw gripper**. The PyBullet prototype now supports analytical FK/IK, smooth joint-space trajectories, Cartesian targeting, physical object contact, pinch grasp, lift and gravity-based release. Next is autonomous pick-and-place, then adaptation of Project Neptune's Talon manipulator in Stonefish.

**Status:** **M0–M3 complete** for the tested simulation scope; **M4 (autonomous pick-and-place with known simulated object pose) is next, not yet implemented**. M3 implementation was fast-forward merged to `main` at [`304fb40`](https://github.com/sidhantjha/neptune-arm-sim/commit/304fb401db97a7a8215ef44c6858a53517b5f60a).

## Quick start

Tested by the developer on **Windows PowerShell**, in the Conda environment `neptune-arm` with Python 3.11 and PyBullet. Work from the repository root. If setting up a new environment, install `pybullet` in the active Python environment.

```powershell
# Run the interactive M2 Cartesian trajectory demo (PyBullet GUI)
python -m src.kinematics_control.demo

# Validate M2 motion system without opening a GUI
python -m src.kinematics_control.validate

# M3: finger actuation, fixed-object contacts, dynamic lift, and multi-mass verification
python -m src.grasping.gripper
python -m src.grasping.contact
python -m src.grasping.lift
python -m src.grasping.validate

# Optional: inspect individual modules
python -m src.common.simulation
python -m src.kinematics_control.kinematics
python -m src.kinematics_control.trajectory
python -m src.kinematics_control.controller

# Historical, monolithic development demo (retained for reference)
python src/run_arm.py
```

Run the GUI demo to change **Target X** and **Target Z** sliders in meters. The controller computes allowed IK solutions, chooses a nearby joint configuration, and follows a quintic, time-scaled joint-space trajectory. Unreachable Cartesian requests are rejected; an invalid request does not erase the prior commanded goal.

## Repository layout

```text
models/
  2_dof_arm.urdf            # Original 2-DOF model (preserved for M2)
  2_dof_gripper.urdf        # M3 adds fixed palm and two prismatic fingers
src/
  run_arm.py                # Legacy integrated M1/M2 development script
  common/
    config.py               # Geometry, physics/motor parameters, model path
    simulation.py           # PyBullet connect, ground, step, disconnect
    robot.py                # URDF loading, pedestal contacts, joint and tip state
  kinematics_control/
    kinematics.py           # Analytical FK / IK and reachability/limit checks
    trajectory.py           # Quintic interpolation for one or two joints
    controller.py           # Stateful IK/trajectory planning and motor commands
    demo.py                 # Preferred interactive M2 GUI
    validate.py             # Headless six-check M2 validation
  grasping/
    gripper.py              # Finger position/opening-width controller
    contact.py              # Test cube, friction, finger contact readings
    lift.py                 # Supported cube, pinch, lift and release
    validate.py             # Six deterministic mass/repeat trials
docs/
  PROJECT.md                # Mission, architecture, constraints, roadmap
  PROGRESS.md               # Verified accomplishments and milestone status
  EXPERIMENTS.md            # Test conditions, results, limitations
  NEXT_STEPS.md             # Exact next action and continuation plan
```

`metrics.py` and duplicate manual demos were intentionally not added. M3 is implemented in `src/grasping/`, reusing M2 `ArmController` and the shared simulation module. The older `src/run_arm.py` is retained as a historical script and has not been deleted. The original M2 URDF and six-check validator remain unchanged.

## Model conventions and scope

- PyBullet **world frame**: X horizontal, Z upward; arm travels in the X–Z plane. This is **not** the Neptune vehicle body convention (+Z down).
- Shoulder pivot: (0, 0, 0.65) m. Links: `L1=0.30 m`, `L2=0.25 m`; both joints rotate about (0, −1, 0), each limited in the URDF to ±2.6 rad.
- TCP in M2: the end of Link 2, obtained by applying a local (0.25, 0, 0) m offset to its world pose.
- Gravity: (0, 0, −9.81) m/s²; physics step: 1/240 s.
- PyBullet position-control commands use a maximum effort setting of 12 N·m and maximum motor velocity of 2 rad/s. The trajectory planner limits its *planned* peak joint speed to 1.5 rad/s and uses a minimum 2 s duration.

Analytical FK:

```text
x = L1*cos(q1) + L2*cos(q1 + q2)
z = 0.65 + L1*sin(q1) + L2*sin(q1 + q2)
```

IK uses a two-branch cosine-law solution, filtered by joint limits. The motor controller then generates quintic, **joint-space** intermediate position commands. Collision responses against the pedestal were manually observed after enabling the two base/link collision pairs; full collision-free planning has **not** been implemented.

## Verified M2 validation

The developer ran `python -m src.kinematics_control.validate` locally after the refactor and reported **6/6 PASS**:

| Check | Result | Final Cartesian error |
|---|---|---:|
| Analytical FK/IK round trip | PASS | Below FK tolerance |
| Target (0.35, 0.85) m | PASS | 0.000001 m |
| Target (0.45, 0.65) m | PASS | 0.000005 m |
| Target (0.25, 1.00) m | PASS | 0.000001 m |
| Target (0.40, 0.90) m | PASS | 0.000003 m |
| Reject unreachable (0.50, 1.10) m | PASS | Not applicable |

Acceptance thresholds in `validate.py`: Cartesian final error ≤0.01 m, joint final error ≤2°, independent FK error ≤0.0001 m. The observed maximum reported final joint error was 0.001°, and the maximum reported FK error was 0.00000007 m. These are **simulation** results, not hardware accuracy guarantees or proof of collision-free trajectories.

## Verified M3 physical grasping

The developer ran M3 on Windows/PyBullet, reporting **all tests PASS**:

- `python -m src.grasping.gripper`: fingers reached 0.0980 m open and 0.0280 m closed.
- `python -m src.grasping.contact`: a **static 40 mm cube** had no initial finger contacts; closing produced a 0.0400 m actual gap, four contact points on each finger and **8.000 N** reported summed normal force per finger.
- `python -m src.grasping.lift`: a **movable 50 g cube** was pinched through frictional contacts, lifted **0.1009 m** and released with **0.1016 m** measured drop; contact counts persisted after lift.
- `python -m src.grasping.validate`: **6/6 PASS**, two separate reset-simulation runs each at **30 g, 50 g, 80 g**, with lifts approximately 0.0973–0.1013 m and drops approximately 0.0975–0.1016 m.
- `python -m src.kinematics_control.validate`: M2 **6/6 PASS** again after M3.

The M3 robot URDF has two revolute arm joints, a fixed palm mount, and two prismatic sliding-finger joints (four actuated DOF). Its symmetric finger controller commands openings between **28 and 98 mm** using up to 8 N motor force per finger, with provisional lateral friction 1.0. **No artificial object attachment is used.**

**Scope:** all M3 tests used one 40 mm cube geometry, fixed starting pose, and two repeats per mass under identical conditions; they do not demonstrate robustness to arbitrary placement, orientation, object geometry, or uncertain friction. The 8 N fixed-object contact reading is not a calibration of grasp force in moving trials.

## Milestones

| Milestone | Status |
|---|---|
| M0 — Repository and PyBullet environment | Complete |
| M1 — 2-DOF URDF, manual sliders, FK comparison | Manual demonstration complete |
| M2 — FK/IK, Cartesian control, quintic trajectories, modular architecture, validation | **Complete: 6/6 headless validation** |
| M3 — Two-finger gripper, contact forces, grasp/lift/release | **Complete: 6/6 repeated-trial validation** |
| M4 — Autonomous pick-and-place using known object pose | **Next; not yet implemented** |
| M5 — Talon arm CAD integration | Planned |
| M6 — Stonefish/Neptune ROV integration | Planned |

## Known limitations

- Joint-space interpolation does not guarantee straight-line TCP motion or collision-free paths.
- Replanning while dragging sliders restarts a quintic path and is position-continuous at the commanded configuration, but may not be velocity/acceleration continuous.
- Model dimensions, masses, inertias and motor settings are provisional examples, not identified Talon hardware parameters.
- No autonomous pick-and-place task state machine, object perception, underwater dynamics, or real-robot control yet.
- The M2 validator checks selected target endpoints and reachability, not all intermediate collisions. M3 tests deterministic grasp/lift/release at a single object pose, not varied placement/friction. The M2 TCP points to Link 2's end, while the M3 grasp center is offset; M4 must account for that transform.

See [project definition](docs/PROJECT.md), [milestone progress](docs/PROGRESS.md), [recorded experiments](docs/EXPERIMENTS.md), and [next steps](docs/NEXT_STEPS.md). The parent vehicle project lives separately at [Project-Lumyn/Project-Neptune](https://github.com/Project-Lumyn/Project-Neptune).
