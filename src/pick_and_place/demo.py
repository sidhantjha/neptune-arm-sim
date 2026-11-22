
import argparse
import math
import time
import pybullet as p

from src.common.config import (
    TIME_STEP,
    REPO_ROOT,
    GRIPPER_TCP_REACH,
)
from src.common.robot import get_joint_angles
from src.common.simulation import (
    start_simulation,
    step_simulation,
    stop_simulation,
)
from src.kinematics_control.kinematics import forward_kinematics
from src.grasping.contact import (
    create_test_cube,
    configure_friction,
)
from src.pick_and_place.task import (
    PickandPlaceTask,
    TIMEOUT,
)


# ======================================
# CREATE SOURCE / DESTINATION PEDESTALS
# ======================================

def create_platform(client, x, top, half_x, color):

    half = [half_x, 0.065, top / 2]

    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half,
        physicsClientId=client
    )

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half,
        rgbaColor=color,
        physicsClientId=client
    )

    return p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[x, 0, top / 2],
        physicsClientId=client
    )


# ======================================
# MEASURE ACTUAL GRIPPER TOOL CENTER
# ======================================

def measured_gripper_center(robot, client):

    centers = []

    for joint in (3, 4):

        link = p.getLinkState(
            robot,
            joint,
            computeForwardKinematics=True,
            physicsClientId=client
        )

        center, _ = p.multiplyTransforms(
            link[4],
            link[5],
            [0.055, 0, 0],
            [0, 0, 0, 1]
        )

        centers.append(center)

    return tuple(
        (centers[0][i] + centers[1][i]) / 2
        for i in range(3)
    )


# ======================================
# RUN ONE COMPLETE PICK-AND-PLACE
# ======================================

def run_episode(mass=0.05, gui=False):

    client = start_simulation(gui=gui)

    try:
        robot = p.loadURDF(
            str(REPO_ROOT / "models" / "2_dof_gripper.urdf"),
            useFixedBase=True,
            physicsClientId=client
        )

        # Initial test configuration only
        p.resetJointState(
            robot, 0, math.radians(55),
            physicsClientId=client
        )
        p.resetJointState(
            robot, 1, math.radians(-45),
            physicsClientId=client
        )

        # Source platform: top at 0.63 m
        create_platform(
            client,
            0.645,
            0.63,
            0.06,
            [0.45, 0.45, 0.50, 1]
        )

        # Destination platform: top at 0.58 m
        destination_support = create_platform(
            client,
            0.49,
            0.58,
            0.035,
            [0.20, 0.65, 0.30, 1]
        )

        # Movable cube from our existing M3 helper
        cube = create_test_cube(
            client, mass=mass
        )

        configure_friction(robot, cube, client)

        # Verify the TCP correction against PyBullet
        q1, q2 = get_joint_angles(robot, client)

        expected = forward_kinematics(
            q1, q2, GRIPPER_TCP_REACH
        )

        measured = measured_gripper_center(
            robot, client
        )

        tcp_error = math.hypot(
            measured[0] - expected[0],
            measured[2] - expected[1]
        )

        print(f"Physical TCP error: {tcp_error:.8f} m")

        if tcp_error > 0.001 or abs(measured[1]) > 0.001:
            raise RuntimeError(
                "Gripper TCP model does not match URDF"
            )

        if gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=1.5,
                cameraYaw=60,
                cameraPitch=-19,
                cameraTargetPosition=[0.45, 0, 0.75],
                physicsClientId=client
            )

        # Create autonomous task
        task = PickandPlaceTask(
            robot,
            cube,
            destination_support,
            client
        )

        max_steps = math.ceil(
            (sum(TIMEOUT.values()) + 2) / TIME_STEP
        )

        for _ in range(max_steps):

            task.update(TIME_STEP)
            step_simulation(client)

            # GUI playback near real-time
            if gui:
                time.sleep(TIME_STEP)

            if task.state in ("SUCCESS", "FAILED"):
                break

        # Evaluate final result
        final_pos = p.getBasePositionAndOrientation(
            cube,
            physicsClientId=client
        )[0]

        error = math.dist(
            final_pos, task.destination
        )

        success = task.state == "SUCCESS"

        print("\nFinal state:", task.state)

        print(
            "Final cube position:",
            tuple(round(v, 4) for v in final_pos)
        )

        print(f"Placement error: {error:.4f} m")

        print(
            "State history:",
            " -> ".join(task.history)
        )

        if task.failure_reason:
            print("Reason:", task.failure_reason)

        print(
            "M4 RESULT:",
            "PASS" if success else "FAIL"
        )

        if gui:
            input(
                "Inspect the simulation, then press Enter to close..."
            )

        return success, error, task.history

    finally:
        stop_simulation(client)


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--gui",
        action="store_true"
    )

    parser.add_argument(
        "--mass",
        type=float,
        default=0.05
    )

    args = parser.parse_args()

    success, _, _ = run_episode(
        mass=args.mass,
        gui=args.gui
    )

    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
