import pybullet as p

# Finger joint indices from our verified URDF
LEFT_FINGER = 3
RIGHT_FINGER = 4

# Physical opening between inner finger surfaces (m)
MIN_OPENING = 0.028
MAX_OPENING = 0.098

FINGER_FORCE = 8.0
FINGER_SPEED = 0.06

class GripperController:
    def __init__(self, robot, client):
        self.robot = robot
        self.client = client

        # verify that the finger joints exist first
        left_check = p.getJointInfo(
            robot, LEFT_FINGER, physicsClientId = client
        )[1].decode()
        if left_check != "left_finger_joint":
            raise RuntimeError("Left finger not found")


    # Function to set gripper distance between inner surfaces
    def set_opening(self, width):
        if (width < MIN_OPENING or width > MAX_OPENING):
            raise ValueError("Out of gripper limits")

        # convert total opening to movement per finger
        displacement = (width - MIN_OPENING) / 2.0

        # now command both fingers symmetrically
        for joint in [LEFT_FINGER,RIGHT_FINGER]:
            p.setJointMotorControl2(
                self.robot,
                jointIndex=joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=displacement,
                force=FINGER_FORCE,
                maxVelocity=FINGER_SPEED,
                physicsClientId=self.client
            )

    # function to read actual gripper distance between inner surfaces
    def get_opening(self):
        left_position = p.getJointState(
            self.robot,
            LEFT_FINGER,
            physicsClientId=self.client
        )[0]

        right_position = p.getJointState(
            self.robot,
            RIGHT_FINGER,
            physicsClientId=self.client
        )[0]

        return MIN_OPENING + left_position + right_position

#standalone motor test
if __name__ == "__main__":

    from src.common.config import TIME_STEP, REPO_ROOT
    from src.common.simulation import (
        start_simulation,
        step_simulation,
        stop_simulation,
    )

    client = start_simulation(gui=False)

    try:
        robot = p.loadURDF(
            str(REPO_ROOT / "models" / "2_dof_gripper.urdf"),
            useFixedBase=True,
            physicsClientId=client
        )

        # Hold shoulder and elbow near zero
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

        # TEST 1: Open
        gripper.set_opening(MAX_OPENING)

        for _ in range(360):
            step_simulation(client)

        opened_width = gripper.get_opening()
        print(f"Opened width: {opened_width:.4f} m")

        # TEST 2: Close
        gripper.set_opening(MIN_OPENING)

        for _ in range(360):
            step_simulation(client)

        closed_width = gripper.get_opening()
        print(f"Closed width: {closed_width:.4f} m")

        passed = (
            abs(opened_width - MAX_OPENING) < 0.002
            and abs(closed_width - MIN_OPENING) < 0.002
        )

        print("Gripper motor test:", "PASS" if passed else "FAIL")

        if not passed:
            raise SystemExit(1)

    finally:
        stop_simulation(client)


