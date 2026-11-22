
import math

from src.common.config import (
    L1,
    L2,
    SHOULDER_HEIGHT,
    JOINT_LIMIT,
)


# ==========================================
# FORWARD KINEMATICS
# ==========================================

def forward_kinematics(q1, q2):
    """
    Joint angles -> end-effector position.

    Inputs:
        q1: shoulder angle (rad)
        q2: elbow angle (rad)

    Returns:
        (x, z) in world coordinates (meters)
    """

    link2_angle = q1 + q2

    x = L1 * math.cos(q1) + L2 * math.cos(link2_angle)

    z = (
        SHOULDER_HEIGHT
        + L1 * math.sin(q1)
        + L2 * math.sin(link2_angle)
    )

    return x, z


# ==========================================
# INVERSE KINEMATICS
# ==========================================

def inverse_kinematics(x_target, z_target):
    """
    End-effector position -> possible joint angles.

    Returns:
        List of valid (q1, q2) solutions in radians.

    Raises:
        ValueError if no valid solution exists.
    """

    # Express target relative to the shoulder
    u = x_target
    v = z_target - SHOULDER_HEIGHT

    # Law of cosines: D = cos(q2)
    D = (
        u**2 + v**2 - L1**2 - L2**2
    ) / (2 * L1 * L2)

    tolerance = 1e-9

    # Check geometric reachability
    if D < -1.0 - tolerance or D > 1.0 + tolerance:
        raise ValueError("Target outside reachable workspace")

    # Handle small floating-point errors
    D = max(-1.0, min(1.0, D))

    # Both possible elbow configurations
    elbow_angle = math.acos(D)

    elbow_candidates = [
        elbow_angle,
        -elbow_angle
    ]

    valid_solutions = []

    for q2 in elbow_candidates:

        # Direction from shoulder to target
        beta = math.atan2(v, u)

        # Angular correction caused by elbow bend
        alpha = math.atan2(
            L2 * math.sin(q2),
            L1 + L2 * math.cos(q2)
        )

        q1 = beta - alpha

        # Reject joint-limit violations
        if abs(q1) > JOINT_LIMIT or abs(q2) > JOINT_LIMIT:
            continue

        # Avoid duplicate configurations
        duplicate = any(
            abs(q1 - old_q1) < tolerance
            and abs(q2 - old_q2) < tolerance
            for old_q1, old_q2 in valid_solutions
        )

        if not duplicate:
            valid_solutions.append((q1, q2))

    if not valid_solutions:
        raise ValueError("No solution within joint limits")

    return valid_solutions


# ==========================================
# MANUAL VERIFICATION
# ==========================================

if __name__ == "__main__":

    q1_test = math.radians(25)
    q2_test = math.radians(-60)

    # FK: angles -> target position
    x, z = forward_kinematics(q1_test, q2_test)

    print(f"Target: X={x:.4f} m, Z={z:.4f} m")

    # IK: target position -> possible angles
    solutions = inverse_kinematics(x, z)

    for i, (q1, q2) in enumerate(solutions):

        check_x, check_z = forward_kinematics(q1, q2)

        error = math.hypot(check_x - x, check_z - z)

        print(
            f"Solution {i + 1}: "
            f"Shoulder={math.degrees(q1):.2f} deg, "
            f"Elbow={math.degrees(q2):.2f} deg, "
            f"Error={error:.8f} m"
        )
