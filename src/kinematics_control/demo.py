
import math
import time

import pybullet as p

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
from src.kinematics_control.controller import ArmController
from src.kinematics_control.kinematics import forward_kinematics


def main():

    client = start_simulation(gui=True)

    try:
        robot = load_robot(client)
        controller = ArmController(robot, client)

        # Cartesian target sliders
        x_slider = p.addUserDebugParameter(
            "Target X (m)", -0.50, 0.50, 0.35,
            physicsClientId=client
        )

        z_slider = p.addUserDebugParameter(
            "Target Z (m)", 0.10, 1.20, 0.85,
            physicsClientId=client
        )

        print("M2 Cartesian trajectory control ready!")

        step = 0

        while p.isConnected(client):

            # Read user target
            target_x = p.readUserDebugParameter(
                x_slider, physicsClientId=client
            )
            target_z = p.readUserDebugParameter(
                z_slider, physicsClientId=client
            )

            # IK and trajectory planning
            target_valid = controller.set_target(
                target_x, target_z
            )

            # Send intermediate motor commands
            controller.update(TIME_STEP)

            # Advance physics
            step_simulation(client)

            # Report performance once per simulated second
            if step % 240 == 0:

                q1, q2 = get_joint_angles(robot, client)
                tip = get_end_effector_position(robot, client)

                expected_x, expected_z = forward_kinematics(
                    q1, q2
                )

                fk_error = math.hypot(
                    tip[0] - expected_x,
                    tip[2] - expected_z
                )

                target_error = math.hypot(
                    tip[0] - target_x,
                    tip[2] - target_z
                )

                joint_error = (
                    math.degrees(controller.q_command[0] - q1),
                    math.degrees(controller.q_command[1] - q2)
                )

                status = "VALID" if target_valid else "UNREACHABLE"

                print(
                    f"\nTarget: ({target_x:.3f}, {target_z:.3f}) "
                    f"| {status}"
                )

                print(
                    f"Position: ({tip[0]:.3f}, {tip[2]:.3f}) "
                    f"| Target error: {target_error:.5f} m"
                )

                print(
                    f"FK error: {fk_error:.8f} m "
                    f"| Joint errors: "
                    f"{joint_error[0]:.2f}, "
                    f"{joint_error[1]:.2f} deg"
                )

                print(
                    f"Trajectory: "
                    f"{controller.elapsed:.2f}/"
                    f"{controller.duration:.2f} s"
                )

            step += 1
            time.sleep(TIME_STEP)

    except KeyboardInterrupt:
        print("\nSimulation stopped.")

    finally:
        stop_simulation(client)


if __name__ == "__main__":
    main()
