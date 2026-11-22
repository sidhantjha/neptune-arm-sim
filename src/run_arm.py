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

# Analytical Froward Kinematics
def forward_kinematics(q1,q2):
    # q1  = shoulder angle (RAD)
    # q2 = Elbow angle relative to shoulder link angle (RAD)
    # returns (x,z) : End-effector world position in meters

    # Link 2 angle in world
    link2_angle = q1 + q2
    # Horizontal position of end effector
    x = L1 * math.cos(q1) + L2 * math.cos(link2_angle)
    z = SHOULDER_HEIGHT + L1 * math.sin(q1) + L2 * math.sin(link2_angle)
    return x,z

# Analytical Inverse Kinematics

# M2.2 - Analytical inverse kinematics

def inverse_kinematics(x_target, z_target):
    """
    Find valid shoulder and elbow angles for a target position.

    Inputs:
        x_target: Desired world X coordinate in meters
        z_target: Desired world Z coordinate in meters

    Returns:
        List of valid (q1, q2) angle pairs in radians.

    Raises:
        ValueError if the target cannot be reached.
    """

    # STEP 1: Express target relative to shoulder
    u = x_target
    v = z_target - SHOULDER_HEIGHT

    # STEP 2: Calculate cos(q2) using the cosine law
    D = (
        u**2 + v**2 - L1**2 - L2**2
    ) / (2 * L1 * L2)

    # STEP 3: Check geometric reachability
    tolerance = 1e-9

    if D < -1.0 - tolerance or D > 1.0 + tolerance:
        raise ValueError("Target is outside the arm's workspace")

    # Protect against tiny floating-point errors
    D = max(-1.0, min(1.0, D))

    # STEP 4: Both possible elbow configurations
    elbow_angle = math.acos(D)

    elbow_candidates = [
        elbow_angle,
        -elbow_angle
    ]

    valid_solutions = []

    # STEP 5: Find shoulder angle for each elbow candidate
    for q2 in elbow_candidates:

        beta = math.atan2(v, u)

        alpha = math.atan2(
            L2 * math.sin(q2),
            L1 + L2 * math.cos(q2)
        )

        q1 = beta - alpha

        # STEP 6: Check actual URDF joint limits
        if abs(q1) > 2.6 or abs(q2) > 2.6:
            continue

        # Avoid returning the same solution twice
        duplicate = any(
            abs(q1 - old_q1) < tolerance
            and abs(q2 - old_q2) < tolerance
            for old_q1, old_q2 in valid_solutions
        )

        if not duplicate:
            valid_solutions.append((q1, q2))

    if not valid_solutions:
        raise ValueError(
            "Target is geometrically reachable, "
            "but violates joint limits"
        )

    return valid_solutions


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
        #Load the robot into the simulation
        urdf_path = (
            Path(__file__).resolve().parents[1]
            / "models"
            / "2_dof_arm.urdf"
        )
        robot = p.loadURDF(
            str(urdf_path),
            useFixedBase = True
        )
        p.setCollisionFilterPair(
            robot, robot,
            -1, 0,
            enableCollision=1
        )

        p.setCollisionFilterPair(
            robot, robot,
            -1, 1,
            enableCollision=1
        )

        # Find our custom robot file
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


        # M2.3 — Cartesian target controls

        x_slider = p.addUserDebugParameter(
            "Target X (meters)",
            -0.50,
            0.50,
            0.35
        )

        z_slider = p.addUserDebugParameter(
            "Target Z (meters)",
            0.10,
            1.20,
            0.85
        )

        print("\nCartesian target sliders created!")

        # M2.2 - Manual IK verification

        # Start from a known joint configuration
        test_q1 = math.radians(25)
        test_q2 = math.radians(-60)

        # Forward kinematics: angles -> position
        target_x, target_z = forward_kinematics(
            test_q1, test_q2
        )

        print("\nIK TEST")
        print(f"Target X: {target_x:.4f} m")
        print(f"Target Z: {target_z:.4f} m")

        # Inverse kinematics: position -> possible angles
        solutions = inverse_kinematics(
            target_x, target_z
        )

        for index, (q1, q2) in enumerate(solutions):

            # Verify every returned solution with FK
            check_x, check_z = forward_kinematics(q1, q2)

            position_error = math.hypot(
                check_x - target_x,
                check_z - target_z
            )

            print(f"\nSolution {index + 1}")
            print(f"Shoulder: {math.degrees(q1):.2f} deg")
            print(f"Elbow: {math.degrees(q2):.2f} deg")
            print(f"Position error: {position_error:.8f} m")

        # Part 3 Motor contorl and physics simulation
        # Start with the robot's current joint configuration.
        last_targets = (
            p.getJointState(robot, 0)[0],
            p.getJointState(robot, 1)[0]
        )
        step = 0
        while p.isConnected():

            # STEP 1: Read desired Cartesian coordinates
            target_x = p.readUserDebugParameter(x_slider)
            target_z = p.readUserDebugParameter(z_slider)

            # STEP 2: Solve inverse kinematics
            try:
                solutions = inverse_kinematics(
                    target_x,
                    target_z
                )

                # STEP 3: Choose the solution closest
                # to our previously commanded joint angles.
                joint_targets = min(
                    solutions,
                    key=lambda solution: (
                        (solution[0] - last_targets[0])**2
                        + (solution[1] - last_targets[1])**2
                    )
                )

                last_targets = joint_targets
                target_valid = True

            except ValueError:
                # If the position is unreachable, keep
                # commanding the last valid configuration.
                joint_targets = last_targets
                target_valid = False

            # STEP 4: Send IK-generated angles to motors
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

                # Measure distance from actual tip to desired target
                target_error = math.hypot(
                    tip[0] - target_x,
                    tip[2] - target_z
                )

                # Compare requested joint positions with actual joint angles
                shoulder_error = math.degrees(joint_targets[0] - q1)
                elbow_error = math.degrees(joint_targets[1] - q2)

                print(
                    f"\nTarget: ({target_x:.3f}, {target_z:.3f}) m"
                )

                if target_valid:
                    print(
                        f"Target position error: {target_error:.5f} m"
                    )
                else:
                    print("Target unreachable — holding last valid command")

                print(
                    f"Joint errors: shoulder={shoulder_error:.2f} deg, "
                    f"elbow={elbow_error:.2f} deg"
                )


                print(
                    f"Shoulder: {math.degrees(q1):.2f} deg | "
                    f"Elbow: {math.degrees(q2):.2f} deg | "
                    f"X: {tip[0]:.3f} m | "
                    f"Z: {tip[2]:.3f} m | "
                    f"FK Error: {error:.6f} m"
                )

                # Read all contact points within the robot
                contacts = p.getContactPoints(
                    bodyA=robot,
                    bodyB=robot
                )

                # Only inspect contacts between the base (-1)
                # and the forearm (Link 2, index 1)
                base_contacts = [
                    c for c in contacts
                    if {c[3], c[4]} == {-1, 1}
                ]

                print("Base/forearm contacts:", len(base_contacts))

                if base_contacts:
                    print(
                        "Minimum contact distance:",
                        min(c[8] for c in base_contacts),
                        "m"
                    )

                # Inspect contacts for both arm links against the pedestal
                contacts = p.getContactPoints(bodyA=robot, bodyB=robot)

                for link_index in [0, 1]:
                    link_contacts = [
                        c for c in contacts
                        if {c[3], c[4]} == {-1, link_index}
                    ]

                    total_force = sum(c[9] for c in link_contacts)

                    print(
                        f"Base/Link {link_index}: "
                        f"{len(link_contacts)} contacts, "
                        f"normal force = {total_force:.3f} N"
                    )

            step += 1

            # Slow down the simulation to approximately real time
            time.sleep(TIME_STEP)

            # Check our robot once ever 240 steps



    finally:
        if p.isConnected(client):
            p.disconnect(client)

if __name__ == "__main__":
    main()

