import pybullet as p

from src.common.simulation import (
    start_simulation,
    step_simulation,
    stop_simulation,
)
from src.grasping.gripper import (
    GripperController,
    LEFT_FINGER,
    RIGHT_FINGER,
    MIN_OPENING,
    MAX_OPENING,
)
from src.common.config import REPO_ROOT

# Creating a test cube to grasp
def create_test_cube(client, mass = 0.0):
    cube_size = 0.04
    half = cube_size / 2.0

    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[half, half, half],
        physicsClientId=client
    )
    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[half, half, half],
        rgbaColor=[1.0, 0.55, 0.1, 1.0],
        physicsClientId=client
    )
    # center cube between the two fingers
    cube = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[0.645, 0.0, 0.65],
        physicsClientId=client
    )
    return cube

# configure contact friction to hold the cube
def configure_friction(robot, cube, client):
    for finger in [LEFT_FINGER, RIGHT_FINGER]:
        p.changeDynamics(
            robot,
            finger,
            lateralFriction=1.0,
            physicsClientId=client
        )

    p.changeDynamics(
        cube,
        -1,
        lateralFriction=1.0,
        physicsClientId=client
    )

# read finger contacts
def get_finger_contacts(robot, cube, client):

    left_contacts = p.getContactPoints(
        bodyA=robot,
        bodyB=cube,
        linkIndexA=LEFT_FINGER,
        physicsClientId=client
    )

    right_contacts = p.getContactPoints(
        bodyA=robot,
        bodyB=cube,
        linkIndexA=RIGHT_FINGER,
        physicsClientId=client
    )

    return left_contacts, right_contacts


# ==========================================
# CONTACT TEST
# ==========================================

def main():

    client = start_simulation(gui=False)

    try:
        robot = p.loadURDF(
            str(REPO_ROOT / "models" / "2_dof_gripper.urdf"),
            useFixedBase=True,
            physicsClientId=client
        )

        # Hold the arm horizontal
        for joint in [0, 1]:
            p.setJointMotorControl2(
                robot,
                jointIndex=joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.0,
                force=12.0,
                maxVelocity=2.0,
                physicsClientId=client
            )

        gripper = GripperController(robot, client)

        # First fully open the gripper
        gripper.set_opening(MAX_OPENING)

        for _ in range(360):
            step_simulation(client)

        # Now introduce the cube
        cube = create_test_cube(client)
        configure_friction(robot, cube, client)

        # Check that open fingers do not touch
        for _ in range(10):
            step_simulation(client)

        left_before, right_before = get_finger_contacts(
            robot, cube, client
        )

        print("Contacts before closing:",
              len(left_before), len(right_before))

        # Close the gripper against the cube
        gripper.set_opening(MIN_OPENING)

        for _ in range(360):
            step_simulation(client)

        # Measure final opening and contacts
        actual_opening = gripper.get_opening()

        left, right = get_finger_contacts(
            robot, cube, client
        )

        left_force = sum(c[9] for c in left)
        right_force = sum(c[9] for c in right)

        print(f"Actual opening: {actual_opening:.4f} m")
        print(f"Left contacts: {len(left)}")
        print(f"Right contacts: {len(right)}")
        print(f"Left normal force: {left_force:.3f} N")
        print(f"Right normal force: {right_force:.3f} N")

        passed = (
            len(left_before) == 0
            and len(right_before) == 0
            and len(left) > 0
            and len(right) > 0
            and left_force > 0
            and right_force > 0
        )

        print(
            "Contact test:",
            "PASS" if passed else "FAIL"
        )

        if not passed:
            raise SystemExit(1)

    finally:
        stop_simulation(client)


if __name__ == "__main__":
    main()
