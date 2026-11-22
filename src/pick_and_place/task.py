import math
import pybullet as p

from src.common.config import GRIPPER_TCP_REACH
from src.common.robot import get_joint_angles
from src.kinematics_control.controller import ArmController
from src.kinematics_control.kinematics import (
    forward_kinematics,
    inverse_kinematics,
)
from src.grasping.gripper import (
    GripperController,
    MIN_OPENING,
    MAX_OPENING,
)
from src.grasping.contact import get_finger_contacts


# Raised HOME keeps the arm clear of the cube
HOME_ANGLES = (math.radians(35), math.radians(-30))

JOINT_TOLERANCE = math.radians(1.0)
TCP_TOLERANCE = 0.012
OPEN_TOLERANCE = 0.002

TIMEOUT = {
    "HOME": 6.0,
    "OPEN": 3.0,
    "APPROACH": 6.0,
    "ALIGN": 6.0,
    "GRASP": 4.0,
    "LIFT": 6.0,
    "TRANSFER": 6.0,
    "PLACE": 6.0,
    "RELEASE": 6.0,
    "RETREAT": 6.0,
}

class PickandPlaceTask:

    def __init__(self, robot, cube, destination_support, client, destination=(0.49, 0.0, 0.60),
    ):
        self.robot = robot
        self.cube = cube
        self.destination_support = destination_support
        self.client = client

        self.arm = ArmController(robot, client)
        self.gripper = GripperController(robot, client)

        # Known ground-truth object position from PyBullet
        self.source = p.getBasePositionAndOrientation(
            cube, physicsClientId=client
        )[0]

        self.destination = destination

        if abs(self.source[1]) > 0.01 or abs(destination[1]) > 0.01:
            raise ValueError("2-DOF arm requires objects near Y = 0")

        self.state = ""
        self.state_time = 0.0
        self.target_tcp = None
        self.failure_reason = ""
        self.history = []

        self._enter("HOME")

    # ======================================
    # FAILURE HANDLING
    # ======================================

    def _fail(self, reason):
        self.failure_reason = reason
        self.state = "FAILED"
        self.history.append("FAILED")
        print("FAILED:", reason)

    # ======================================
    # MOVE THE ACTUAL GRIPPER CENTER
    # ======================================

    def _move_tcp(self, x, z):

        try:
            candidates = inverse_kinematics(
                x, z, GRIPPER_TCP_REACH
            )
        except ValueError as exc:
            self._fail(
                f"Unreachable TCP target ({x:.3f}, {z:.3f}): {exc}"
            )
            return

        # Prefer negative elbow angle to keep
        # a consistent arm configuration
        candidates = [
            q for q in candidates if q[1] <= 1e-6
        ]

        if not candidates:
            self._fail("No elbow-negative joint solution")
            return

        goal = min(
            candidates,
            key=lambda q: sum(
                (q[i] - self.arm.q_command[i]) ** 2
                for i in range(2)
            )
        )

        if not self.arm.set_joint_target(*goal):
            self._fail("Arm rejected joint target")
            return

        self.target_tcp = (x, z)

    # ======================================
    # ENTER A NEW STATE
    # ======================================

    def _enter(self, state):

        self.state = state
        self.state_time = 0.0
        self.history.append(state)

        print("STATE:", state)

        sx, _, sz = self.source
        dx, _, dz = self.destination

        if state == "HOME":

            self.arm.set_joint_target(*HOME_ANGLES)

            self.target_tcp = forward_kinematics(
                *HOME_ANGLES, GRIPPER_TCP_REACH
            )

        elif state == "OPEN":

            self.gripper.set_opening(MAX_OPENING)

        elif state == "APPROACH":

            # Move above and slightly behind the cube
            self._move_tcp(sx - 0.045, sz + 0.13)

        elif state == "ALIGN":

            # Move the gripper center onto the cube center
            self._move_tcp(sx, sz)

        elif state == "GRASP":

            self.gripper.set_opening(MIN_OPENING)

        elif state == "LIFT":

            # Approximately 10 cm above initial cube height
            self._move_tcp(sx - 0.008, sz + 0.10)

        elif state == "TRANSFER":

            # High clearance waypoint
            self._move_tcp(0.55, 0.93)

        elif state == "PLACE":

            # Approach 3 cm above final resting position
            self._move_tcp(dx, dz + 0.03)

        elif state == "RELEASE":

            self.gripper.set_opening(MAX_OPENING)

        elif state == "RETREAT":

            self._move_tcp(0.55, 0.93)

    # ======================================
    # CHECK ARM MOTION COMPLETE
    # ======================================

    def _arm_reached(self):

        if self.arm.elapsed < self.arm.duration:
            return False

        q = get_joint_angles(self.robot, self.client)

        actual = forward_kinematics(
            *q, GRIPPER_TCP_REACH
        )

        position_ok = (
            math.dist(actual, self.target_tcp)
            < TCP_TOLERANCE
        )

        joint_ok = max(
            abs(q[i] - self.arm.q_goal[i])
            for i in range(2)
        ) < JOINT_TOLERANCE

        return position_ok and joint_ok

    # ======================================
    # CHECK PHYSICAL GRASP
    # ======================================

    def _has_grasp(self):

        left, right = get_finger_contacts(
            self.robot, self.cube, self.client
        )

        return (
            bool(left)
            and bool(right)
            and sum(c[9] for c in left) > 0.0
            and sum(c[9] for c in right) > 0.0
        )

    def _cube_position(self):

        return p.getBasePositionAndOrientation(
            self.cube,
            physicsClientId=self.client
        )[0]

    # ======================================
    # VERIFY OBJECT PLACEMENT
    # ======================================

    def _placed(self):

        pos = self._cube_position()

        velocity, _ = p.getBaseVelocity(
            self.cube,
            physicsClientId=self.client
        )

        contacts = p.getContactPoints(
            bodyA=self.cube,
            bodyB=self.destination_support,
            physicsClientId=self.client,
        )

        xy_error = math.hypot(
            pos[0] - self.destination[0],
            pos[1] - self.destination[1]
        )

        z_error = abs(
            pos[2] - self.destination[2]
        )

        speed = math.sqrt(
            sum(v * v for v in velocity)
        )

        return (
            bool(contacts)
            and xy_error < 0.04
            and z_error < 0.04
            and speed < 0.10
        )

    # ======================================
    # AUTONOMOUS STATE MACHINE
    # ======================================

    def update(self, dt):

        if self.state in ("SUCCESS", "FAILED"):
            return

        # Keep running the existing M2 motor controller
        self.arm.update(dt)
        self.state_time += dt

        state = self.state

        if self.state_time > TIMEOUT[state]:
            self._fail(f"{state} timeout")
            return

        if state == "HOME":

            if self._arm_reached():
                self._enter("OPEN")

        elif state == "OPEN":

            error = abs(
                self.gripper.get_opening() - MAX_OPENING
            )

            if error < OPEN_TOLERANCE:
                self._enter("APPROACH")

        elif state == "APPROACH":

            if self._arm_reached():
                self._enter("ALIGN")

        elif state == "ALIGN":

            if self._arm_reached():
                self._enter("GRASP")

        elif state == "GRASP":

            if (
                self.state_time > 0.8
                and self._has_grasp()
            ):
                self._enter("LIFT")

        elif state == "LIFT":

            if self._arm_reached():

                height = (
                    self._cube_position()[2]
                    - self.source[2]
                )

                if height > 0.06 and self._has_grasp():
                    self._enter("TRANSFER")
                else:
                    self._fail("Cube not securely lifted")

        elif state == "TRANSFER":

            if self._arm_reached():

                if self._has_grasp():
                    self._enter("PLACE")
                else:
                    self._fail("Cube lost during transfer")

        elif state == "PLACE":

            if self._arm_reached():

                if self._has_grasp():
                    self._enter("RELEASE")
                else:
                    self._fail("Cube lost before release")

        elif state == "RELEASE":

            opened = abs(
                self.gripper.get_opening() - MAX_OPENING
            ) < OPEN_TOLERANCE

            if (
                self.state_time > 1.0
                and opened
                and self._placed()
            ):
                self._enter("RETREAT")

        elif state == "RETREAT":

            if self._arm_reached():

                if self._placed():
                    self._enter("SUCCESS")
                else:
                    self._fail("Cube moved after retreat")
