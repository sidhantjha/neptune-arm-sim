
import math
import pybullet as p

from src.common.config import (
    MOTOR_FORCE,
    MOTOR_MAX_VELOCITY,
    MIN_TRAJECTORY_DURATION,
    PLANNED_SPEED_LIMIT,
    JOINT_LIMIT,
)

from src.common.robot import get_joint_angles

from src.kinematics_control.kinematics import inverse_kinematics
from src.kinematics_control.trajectory import quintic_joint_targets


class ArmController:

    def __init__(self, robot, client):

        self.robot = robot
        self.client = client

        # Read the current actual joint configuration
        current_angles = get_joint_angles(robot, client)

        # Initial trajectory state
        self.q_start = current_angles
        self.q_goal = current_angles
        self.q_command = current_angles

        self.elapsed = 0.0
        self.duration = MIN_TRAJECTORY_DURATION

        self.last_request = None
        self.target_valid = False

    # ======================================
    # SET A NEW CARTESIAN TARGET
    # ======================================

    def set_target(self, x_target, z_target):

        request = (x_target, z_target)

        # Avoid replanning for an unchanged target
        if self.last_request is not None:

            change = math.dist(request, self.last_request)

            if change <= 0.001:
                return self.target_valid

        self.last_request = request

        # Calculate possible joint configurations
        try:
            solutions = inverse_kinematics(
                x_target,
                z_target
            )

        except ValueError:
            self.target_valid = False
            return False

        # Choose the solution closest to the current command
        new_goal = min(
            solutions,
            key=lambda solution: sum(
                (solution[i] - self.q_command[i])**2
                for i in range(2)
            )
        )

        # Create a new trajectory
        return self._plan_joint_goal(new_goal)

    def set_joint_target(self,q1,q2):
        # Command a joint configuration directly
        if abs(q1) > JOINT_LIMIT or abs(q2) > JOINT_LIMIT:
            return False
        self.last_request = None
        return self._plan_joint_goal((q1, q2))

    def _plan_joint_goal(self,goal):
        self.q_start = self.q_command
        self.q_goal = goal
        self.elapsed = 0.0

        max_delta = max(
            abs(self.q_goal[i] - self.q_start[i])
            for i in range(2)
        )

        self.duration = max(
            MIN_TRAJECTORY_DURATION,
            1.875 * max_delta / PLANNED_SPEED_LIMIT
        )

        self.target_valid = True
        return True

    # ======================================
    # ADVANCE CONTROLLER BY ONE TIME STEP
    # ======================================

    def update(self, dt):

        if dt <= 0:
            raise ValueError("Time step must be positive")

        self.elapsed = min(
            self.elapsed + dt,
            self.duration
        )

        # Calculate the next intermediate joint angles
        self.q_command = quintic_joint_targets(
            self.q_start,
            self.q_goal,
            self.elapsed,
            self.duration
        )

        # Send commands to PyBullet motors
        for joint_index, target in enumerate(self.q_command):

            p.setJointMotorControl2(
                self.robot,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target,
                force=MOTOR_FORCE,
                maxVelocity=MOTOR_MAX_VELOCITY,
                physicsClientId=self.client
            )

        return self.q_command


# ======================================
# STANDALONE MANUAL VERIFICATION
# ======================================

if __name__ == "__main__":

    from src.common.config import TIME_STEP
    from src.common.simulation import (
        start_simulation,
        step_simulation,
        stop_simulation,
    )
    from src.common.robot import (
        load_robot,
        get_end_effector_position,
    )

    client = start_simulation(gui=False)

    try:
        robot = load_robot(client)
        controller = ArmController(robot, client)

        x_target = 0.35
        z_target = 0.85

        print("Target accepted:",
              controller.set_target(x_target, z_target))

        # Simulate 3 seconds
        for _ in range(720):
            controller.update(TIME_STEP)
            step_simulation(client)

        tip = get_end_effector_position(robot, client)

        error = math.hypot(
            tip[0] - x_target,
            tip[2] - z_target
        )

        print(f"Final X: {tip[0]:.4f} m")
        print(f"Final Z: {tip[2]:.4f} m")
        print(f"Position error: {error:.6f} m")

    finally:
        stop_simulation(client)
