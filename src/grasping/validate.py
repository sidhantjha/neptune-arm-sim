
import math
import pybullet as p

from src.common.config import TIME_STEP, REPO_ROOT
from src.common.simulation import (
    start_simulation,
    step_simulation,
    stop_simulation,
)
from src.kinematics_control.controller import ArmController
from src.kinematics_control.kinematics import forward_kinematics
from src.grasping.gripper import (
    GripperController,
    MAX_OPENING,
    MIN_OPENING,
)
from src.grasping.contact import (
    create_test_cube,
    configure_friction,
    get_finger_contacts,
)
from src.grasping.lift import create_support, cube_position


# Run multiple independent grasp experiments
TEST_MASSES = [0.03, 0.05, 0.08]
REPEATS = 2


def advance(client, arm, steps):
    for _ in range(steps):
        arm.update(TIME_STEP)
        step_simulation(client)


def run_trial(mass):

    # Fresh simulation for each trial
    client = start_simulation(gui=False)

    try:
        robot = p.loadURDF(
            str(REPO_ROOT / "models" / "2_dof_gripper.urdf"),
            useFixedBase=True,
            physicsClientId=client
        )

        arm = ArmController(robot, client)
        gripper = GripperController(robot, client)

        # Initialize arm and open fingers
        arm.set_target(0.55, 0.65)
        gripper.set_opening(MAX_OPENING)
        advance(client, arm, 360)

        opened = abs(
            gripper.get_opening() - MAX_OPENING
        ) < 0.002

        # Place movable cube on support
        create_support(client)
        cube = create_test_cube(client, mass=mass)
        configure_friction(robot, cube, client)

        advance(client, arm, 120)

        left, right = get_finger_contacts(robot, cube, client)

        initially_clear = len(left) == 0 and len(right) == 0

        # Close fingers onto cube
        gripper.set_opening(MIN_OPENING)
        advance(client, arm, 360)

        left, right = get_finger_contacts(robot, cube, client)

        grasped = (
            len(left) > 0
            and len(right) > 0
            and sum(c[9] for c in left) > 0
            and sum(c[9] for c in right) > 0
        )

        before = cube_position(cube, client)

        # Lift using M2 arm controller
        x, z = forward_kinematics(math.radians(9), 0.0)

        if not arm.set_target(x, z):
            return False, 0.0, 0.0

        steps = math.ceil((arm.duration + 0.5) / TIME_STEP)
        advance(client, arm, steps)

        after_lift = cube_position(cube, client)

        lift_height = after_lift[2] - before[2]

        left, right = get_finger_contacts(robot, cube, client)

        lifted = (
            lift_height > 0.04
            and len(left) > 0
            and len(right) > 0
        )

        # Release the cube
        gripper.set_opening(MAX_OPENING)
        advance(client, arm, 360)

        after_release = cube_position(cube, client)

        drop_distance = (
            after_lift[2] - after_release[2]
        )

        released = drop_distance > 0.04

        passed = all([
            opened,
            initially_clear,
            grasped,
            lifted,
            released,
        ])

        return passed, lift_height, drop_distance

    finally:
        stop_simulation(client)


def main():

    results = []

    print("\n=== M3 GRIPPER VALIDATION ===\n")

    for mass in TEST_MASSES:
        for attempt in range(1, REPEATS + 1):

            passed, lift, drop = run_trial(mass)

            results.append(passed)

            status = "PASS" if passed else "FAIL"

            print(
                f"Mass={mass:.3f} kg "
                f"| Trial={attempt} "
                f"| Lift={lift:.4f} m "
                f"| Drop={drop:.4f} m "
                f"| {status}"
            )

    passed_count = sum(results)
    total = len(results)

    print("\n==============================")
    print(f"M3 VALIDATION: {passed_count}/{total} PASSED")
    print("==============================")

    if not all(results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
