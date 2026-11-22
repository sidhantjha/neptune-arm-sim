import argparse
import math
import time
from pathlib import Path
import pybullet as p
import pybullet_data

# Robot geometry in meters
L1 = 0.30
L2 = 0.25
SHOULDER_HEIGHT = 0.65
TIME_STEP = 1.0 / 240.0

# Find the end of forearm (Link 2) tip in world coordinates
def get_end_effector_position(robot):
    state = p.getLinkState(
        robot, 1, computeForwardKinematics=True
    )
    link_pos = state[4] # world position of Link2's reference frame
    link_orientation = state[5] # orientation of Link 2's reference frame, expressed as quaternion
    end_effector_pos, end_effector_orientation = p.multiplyTransforms(
        link_pos,
        link_orientation,
        [L2,0,0],
        [0,0,0,1],
    )
    return end_effector_pos

def main():
    client = p.connect(p.GUI)
    if client<0 :
        raise RuntimeError("Couldn't connect to Pybullet")
    try:
        p.setAdditionalSearchPath(
            pybullet_data.getDataPath()
        )

        # Configure physics world
        p.setGravity(0,0,-9.81)
        p.setTimeStep(TIME_STEP)
        # Load the ground
        ground = p.loadURDF("plane.urdf")
        # Find our custom robot file
        urdf_path = (
            Path(__file__).resolve().parents[1]
            / "models"
            / "2_dof_arm.urdf"
        )
        #Load the robot into the simulation
        robot = p.loadURDF(
            str(urdf_path),
            useFixedBase = True
        )
        print("Robot loaded successfully!")
        print("Robot ID:", robot)
        print("Number of joints:", p.getNumJoints(robot))

        # PART 2 - Inspect the robot's joints
        num_joints = p.getNumJoints(robot)
        assert num_joints == 2, "Expected exactly 2 joints"

        for joint_index in range(num_joints):
            # Get joint configuration info from the URDF
            joint_info = p.getJointInfo(robot, joint_index)
            joint_name = joint_info[1].decode("utf-8")
            joint_type = joint_info[2]
            lower_limit = joint_info[8]
            upper_limit = joint_info[9]
            joint_axis = joint_info[13]
            print(f"\nJoint {joint_index}: {joint_name}")
            print(f"Type: {joint_type}")
            print(f"Axis: {joint_axis}")
            print(f"Limits: [{lower_limit:.2f}, {upper_limit:.2f}] rad")
            assert joint_type == p.JOINT_REVOLUTE
            # Read current simulated joint state
            joint_state = p.getJointState(robot, joint_index)
            current_angle = joint_state[0]
            current_velocity = joint_state[1]
            print(f"Current angle: {math.degrees(current_angle):.2f} deg")
            print(f"Current velocity: {current_velocity:.3f} rad/s")

        shoulder_slider = p.addUserDebugParameter(
            "Shoulder angle (degrees)",
            -140,
            140,
            25
        )

        elbow_slider = p.addUserDebugParameter(
            "Elbow angle (degrees)",
            -140,
            140,
            -60
        )

        print("\nJoint sliders created!")
        # Part 3 Motor contorl and physics simulation
        step = 0
        while p.isConnected():
            # Read desired joint angles from the sliders
            shoulder_deg = p.readUserDebugParameter(shoulder_slider)
            elbow_deg = p.readUserDebugParameter(elbow_slider)

            # Convert deg to rad
            q1_target = math.radians(shoulder_deg)
            q2_target = math.radians(elbow_deg)
            joint_targets = [q1_target, q2_target]

            # Send desired positions to joint motors
            for joint_index, target in enumerate(joint_targets):
                p.setJointMotorControl2(
                    robot,
                    jointIndex=joint_index,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target,
                    force=12.0,
                    maxVelocity=2.0
                )
            # Advanced physics simulation
            p.stepSimulation()
            if step % 240 == 0:

                # Read actual joint angles
                q1 = p.getJointState(robot, 0)[0]
                q2 = p.getJointState(robot, 1)[0]

                # Get end-effector position from PyBullet
                tip = get_end_effector_position(robot)

                # Calculate expected position using our FK equations
                expected_x = (
                    L1 * math.cos(q1)
                    + L2 * math.cos(q1 + q2)
                )

                expected_z = (
                    SHOULDER_HEIGHT
                    + L1 * math.sin(q1)
                    + L2 * math.sin(q1 + q2)
                )

                # Calculate error between PyBullet and our mathematics
                error = math.hypot(
                    tip[0] - expected_x,
                    tip[2] - expected_z
                )

                print(
                    f"Shoulder: {math.degrees(q1):.2f} deg | "
                    f"Elbow: {math.degrees(q2):.2f} deg | "
                    f"X: {tip[0]:.3f} m | "
                    f"Z: {tip[2]:.3f} m | "
                    f"FK Error: {error:.6f} m"
                )

            step += 1

            # Slow down the simulation to approximately real time
            time.sleep(TIME_STEP)

            # Check our robot once ever 240 steps



    finally:
        p.disconnect()

if __name__ == "__main__":
    main()

