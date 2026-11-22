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
from src.grasping.gripper import GripperController, MAX_OPENING, MIN_OPENING
from src.grasping.contact import (
    create_test_cube,
    configure_friction,
    get_finger_contacts,
)


# create a support for the cube
def create_support(client):
    half = [0.11, 0.11, 0.10]

    shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half,
        physicsClientId=client
    )

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half,
        rgbaColor=[0.45, 0.45, 0.50, 1],
        physicsClientId=client
    )

    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=shape,
        baseVisualShapeIndex=visual,
        basePosition=[0.645, 0, 0.53],
        physicsClientId=client
    )

# measure cube positon
def cube_position(cube, client):
    position, _ = p.getBasePositionAndOrientation(
        cube,
        physicsClientId=client
    )
    return position

# full grasp and lift experiment
def main():

    client = start_simulation(gui=False)

    try:
        robot = p.loadURDF(
            str(REPO_ROOT / "models" / "2_dof_gripper.urdf"),
            useFixedBase=True,
            physicsClientId=client
        )

        arm = ArmController(robot, client)
        gripper = GripperController(robot, client)

        # Hold the arm at its initial position
        arm.set_target(0.55, 0.65)

        # Open the fingers
        gripper.set_opening(MAX_OPENING)

        for _ in range(360):
            arm.update(TIME_STEP)
            step_simulation(client)

        # Add a support and a movable 50 g cube
        create_support(client)
        cube = create_test_cube(client, mass=0.05)

        configure_friction(robot, cube, client)

        # Allow cube to settle on the support
        for _ in range(120):
            arm.update(TIME_STEP)
            step_simulation(client)

        # STEP 1: Close around the cube
        gripper.set_opening(MIN_OPENING)

        for _ in range(360):
            arm.update(TIME_STEP)
            step_simulation(client)

        left, right = get_finger_contacts(robot, cube, client)

        grasped = len(left) > 0 and len(right) > 0

        before_lift = cube_position(cube, client)

        print("Grasp contacts:", len(left), len(right))
        print(f"Gripper opening: {gripper.get_opening():.4f} m")

        if not grasped:
            print("FAIL: Both fingers did not contact the cube")
            raise SystemExit(1)

        # STEP 2: Move the arm upward
        # Rotate the entire arm about 9 degrees
        q1_goal = math.radians(9)
        q2_goal = 0.0

        target_x, target_z = forward_kinematics(
            q1_goal, q2_goal
        )

        if not arm.set_target(target_x, target_z):
            raise RuntimeError("Lift target rejected by IK")

        steps = math.ceil(
            (arm.duration + 0.5) / TIME_STEP
        )

        for _ in range(steps):
            arm.update(TIME_STEP)
            step_simulation(client)

        after_lift = cube_position(cube, client)

        lift_height = after_lift[2] - before_lift[2]

        left, right = get_finger_contacts(robot, cube, client)

        lifted = (
            lift_height > 0.04
            and len(left) > 0
            and len(right) > 0
        )

        print(f"Cube height before: {before_lift[2]:.4f} m")
        print(f"Cube height after: {after_lift[2]:.4f} m")
        print(f"Lift height: {lift_height:.4f} m")
        print("Contacts after lift:", len(left), len(right))
        print("Lift test:", "PASS" if lifted else "FAIL")

        # STEP 3: Release the object
        gripper.set_opening(MAX_OPENING)

        for _ in range(360):
            arm.update(TIME_STEP)
            step_simulation(client)

        after_release = cube_position(cube, client)

        drop_distance = after_lift[2] - after_release[2]

        released = drop_distance > 0.04

        print(f"Drop distance: {drop_distance:.4f} m")
        print("Release test:", "PASS" if released else "FAIL")

        # Final result
        passed = grasped and lifted and released

        print("\n==========================")
        print("M3.4:", "PASS" if passed else "FAIL")
        print("==========================")

        if not passed:
            raise SystemExit(1)

    finally:
        stop_simulation(client)


if __name__ == "__main__":
    main()

