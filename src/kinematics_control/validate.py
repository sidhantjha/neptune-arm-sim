
import math

from src.common.config import TIME_STEP
from src.common.simulation import (
    start_simulation,
    step_simulation,
    stop_simulation,
)
from src.common.robot import (
    load_robot,
    get_joint_angles,
    get_end_effector_position,
)
from src.kinematics_control.kinematics import (
    forward_kinematics,
    inverse_kinematics,
)
from src.kinematics_control.controller import ArmController


POSITION_TOLERANCE = 0.01       # 1 cm
JOINT_TOLERANCE_DEG = 2.0       # 2 degrees
FK_TOLERANCE = 0.0001           # 0.1 mm


def validate_target(controller, robot, client, name, x, z):

    accepted = controller.set_target(x, z)

    if not accepted:
        print(f"{name}: FAIL - target rejected")
        return False

    # Run the full trajectory plus settling time
    simulation_duration = controller.duration + 0.75

    num_steps = math.ceil(simulation_duration / TIME_STEP)

    for _ in range(num_steps):
        controller.update(TIME_STEP)
        step_simulation(client)

    # Measure actual robot state
    q1, q2 = get_joint_angles(robot, client)
    tip = get_end_effector_position(robot, client)

    # Cartesian target error
    position_error = math.hypot(
        tip[0] - x,
        tip[2] - z
    )

    # Independent FK check
    fk_x, fk_z = forward_kinematics(q1, q2)

    fk_error = math.hypot(
        tip[0] - fk_x,
        tip[2] - fk_z
    )

    # Maximum joint tracking error
    joint_error_deg = max(
        abs(math.degrees(controller.q_goal[0] - q1)),
        abs(math.degrees(controller.q_goal[1] - q2))
    )

    passed = (
        position_error <= POSITION_TOLERANCE
        and fk_error <= FK_TOLERANCE
        and joint_error_deg <= JOINT_TOLERANCE_DEG
    )

    status = "PASS" if passed else "FAIL"

    print(
        f"{name}: {status}\n"
        f"  Target: ({x:.3f}, {z:.3f}) m\n"
        f"  Actual: ({tip[0]:.3f}, {tip[2]:.3f}) m\n"
        f"  Position error: {position_error:.6f} m\n"
        f"  FK error: {fk_error:.8f} m\n"
        f"  Max joint error: {joint_error_deg:.3f} deg\n"
        f"  Trajectory duration: {controller.duration:.2f} s"
    )

    return passed


def main():

    client = start_simulation(gui=False)
    results = []

    try:
        robot = load_robot(client)
        controller = ArmController(robot, client)

        print("\n=== M2 SYSTEM VALIDATION ===\n")

        # TEST 1: Analytical FK/IK consistency
        q1_test = math.radians(25)
        q2_test = math.radians(-60)

        x, z = forward_kinematics(q1_test, q2_test)
        solutions = inverse_kinematics(x, z)

        ik_passed = any(
            math.hypot(
                forward_kinematics(q1, q2)[0] - x,
                forward_kinematics(q1, q2)[1] - z
            ) < FK_TOLERANCE
            for q1, q2 in solutions
        )

        results.append(ik_passed)
        print("FK/IK round trip:", "PASS" if ik_passed else "FAIL")

        # TESTS 2-5: Physical target reaching
        targets = [
            ("Initial reach", 0.35, 0.85),
            ("Horizontal reach", 0.45, 0.65),
            ("Raised reach", 0.25, 1.00),
            ("Final reach", 0.40, 0.90),
        ]

        for name, x, z in targets:
            print()
            passed = validate_target(
                controller, robot, client, name, x, z
            )
            results.append(passed)

        # TEST 6: Unreachable target rejection
        previous_goal = controller.q_goal

        accepted = controller.set_target(0.50, 1.10)

        rejected_correctly = (
            not accepted
            and controller.q_goal == previous_goal
        )

        results.append(rejected_correctly)

        print(
            "\nUnreachable target:",
            "PASS" if rejected_correctly else "FAIL"
        )

        # FINAL SUMMARY
        passed_count = sum(results)
        total_count = len(results)

        print("\n==============================")
        print(f"M2 VALIDATION: {passed_count}/{total_count} PASSED")
        print("==============================")

        if not all(results):
            raise SystemExit(1)

        print("M2 motion system validated successfully!")

    finally:
        stop_simulation(client)


if __name__ == "__main__":
    main()
