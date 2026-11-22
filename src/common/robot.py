
import pybullet as p

from src.common.config import (
    ARM_URDF,
    L2,
)


# ==========================================
# LOAD THE ROBOT
# ==========================================

def load_robot(client):

    # Load the robot from its URDF file
    robot = p.loadURDF(
        str(ARM_URDF),
        useFixedBase=True,
        physicsClientId=client
    )

    # Enable collisions between the pedestal
    # and the two moving arm links
    for link_index in [0, 1]:

        p.setCollisionFilterPair(
            robot,
            robot,
            -1,
            link_index,
            enableCollision=1,
            physicsClientId=client
        )

    # Verify our expected two-joint model
    num_joints = p.getNumJoints(
        robot,
        physicsClientId=client
    )

    if num_joints != 2:
        raise RuntimeError("Expected a 2-DOF robot")

    return robot


# ==========================================
# READ ACTUAL JOINT ANGLES
# ==========================================

def get_joint_angles(robot, client):

    q1 = p.getJointState(
        robot, 0,
        physicsClientId=client
    )[0]

    q2 = p.getJointState(
        robot, 1,
        physicsClientId=client
    )[0]

    return q1, q2


# ==========================================
# READ ACTUAL END-EFFECTOR POSITION
# ==========================================

def get_end_effector_position(robot, client):

    state = p.getLinkState(
        robot,
        1,
        computeForwardKinematics=True,
        physicsClientId=client
    )

    # World pose of Link 2's reference frame
    link_pos = state[4]
    link_orientation = state[5]

    # Transform the local tip offset into world coordinates
    tip_pos, _ = p.multiplyTransforms(
        link_pos,
        link_orientation,
        [L2, 0, 0],
        [0, 0, 0, 1]
    )

    return tip_pos


# ==========================================
# READ CONTACTS WITH THE PEDESTAL
# ==========================================

def get_base_contacts(robot, client):

    contacts = p.getContactPoints(
        bodyA=robot,
        bodyB=robot,
        physicsClientId=client
    )

    # Keep contacts between the fixed base (-1)
    # and either moving link (0 or 1)
    base_contacts = [
        c for c in contacts
        if (
            (c[3] == -1 and c[4] in [0, 1])
            or
            (c[4] == -1 and c[3] in [0, 1])
        )
    ]

    return base_contacts
