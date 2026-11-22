# Experiments and Engineering Findings

## Purpose
Track meaningful tests, parameters, outcomes, failures, and lessons. Results explicitly marked as user-observed are grounded in screenshots/terminal output, not independently rerun by the assistant.

## Experiment Index
| ID | Experiment | Result |
|---|---|---|
| EXP-001 | PyBullet DIRECT installation smoke check | PASS — user observed |
| EXP-002 | Two-joint GUI control and FK match | PASS — manual observation |
| EXP-003 | Automated M1 smoke/regression suite | SKIPPED — user preference |

## EXP-001 — PyBullet DIRECT setup
Date: 2026-11-21
Objective: Check runtime import and connection.
Configuration: Windows PowerShell; Conda environment `neptune-arm`; Python 3.11; PyBullet build reported Nov 21, 2026.
Procedure: `python -c "import pybullet as p; c=p.connect(p.DIRECT); print('PyBullet works!', c); p.disconnect()"`
Observed: `PyBullet works! 0`.
Result: **PASS (user-observed)**.

## EXP-002 — M1 manual articulated arm and analytical FK
Date: 2026-11-21
Implementation commit: `35bb5551b2de96f78bf917479e97b40462d8dc81`
Objective: Confirm URDF loads; sliders move both revolute joints; analytical FK matches PyBullet tip transform.
Configuration:
- URDF: `models/2_dof_arm.urdf`; script: `src/run_arm.py`.
- Fixed base, first/second link lengths 0.30/0.25 m, shoulder height 0.65 m.
- Joint axis (0,-1,0), URDF joint limits ±2.6 rad.
- Gravity (0,0,-9.81) m/s²; timestep 1/240 s.
- PyBullet position control; command max effort 12 N·m and max velocity 2 rad/s.
- GUI sliders for shoulder and elbow, values expressed in degrees.
Procedure: User ran `python src/run_arm.py`, manipulated both sliders, inspected printed joint states and FK error.
Observed:
- Robot ID 1; two revolute joints shoulder and elbow.
- Joint angles responded to slider changes.
- Shoulder 25.00°, elbow -60.00° -> tip X=0.477 m, Z=0.633 m, reported FK error=0.000000 m.
- Shoulder 19.16°, elbow 24.21° -> X=0.465 m, Z=0.920 m, reported FK error=0.000000 m.
- Shoulder 19.16°, elbow -20.63° -> X=0.533 m, Z=0.742 m, reported FK error=0.000000 m.
- GUI screenshot showed the arm and parameter sliders.
Result: **PASS (manual, user-observed)**; error is reported to six decimals, not a precise numeric bound.
What worked: URDF loading, index/type inspection, position-controlled joint motion, mathematical FK verification.
Limitations: No automated assertion/regression, no systematic joint tracking metric, no physical-grasp test. FK match does not establish real motor model fidelity.
Incidents: Initial `p.gui` raised `AttributeError`; fixed by using `p.GUI`. Closing/interrupting continuous GUI loop yielded a `KeyboardInterrupt` during `time.sleep`; not a demonstrated simulation failure.

## EXP-003 — Automated testing
Status: **SKIPPED**, by user request; do not claim tests passed.
Revisit if the project begins regression-sensitive changes.

## Next Experiments
- EXP-004: Compare standalone analytical FK against PyBullet across manually selected joint configurations and measure joint tracking error.
- EXP-005: Analytical 2R IK solver reaches feasible requested (x,z) target within joint limits.
- EXP-006: Time-parameterized motion to targets without extreme joint speed jumps.
- EXP-007: Contact-based pinch grasp and lift with two-finger gripper.

## Rejected / Deferred Approaches
- Full-featured arm and Talon CAD import before initial generic validation: deferred intentionally.
- Automated M1 smoke suite: deferred at user request, not a technical rejection.

## Open Questions
- Choose M2 target coordinate convention and joint-angle branch selection policy.
- Define M2 position tolerance and joint tracking metrics.
- Choose M3 gripper actuation linkage, contact material/friction, object mass.
- Define M4 trial set, failure criteria, and success-rate target.
