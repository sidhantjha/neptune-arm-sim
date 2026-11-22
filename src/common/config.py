
from pathlib import Path

# =========================
# ROBOT GEOMETRY
# =========================

L1 = 0.30                 # First link length (m)
L2 = 0.25                 # Second link length (m)
SHOULDER_HEIGHT = 0.65    # Shoulder height in world frame (m)

JOINT_LIMIT = 2.6         # Joint limits: +/- 2.6 radians

# Gripper grasp center relative to elbow joint
GRIPPER_MOUNT_X = 0.27
FINGER_JOINT_X = 0.02
FINGER_CENTER_X = 0.055

GRIPPER_TCP_REACH = (
    GRIPPER_MOUNT_X
    + FINGER_JOINT_X
    + FINGER_CENTER_X
)


# =========================
# PHYSICS CONFIGURATION
# =========================

TIME_STEP = 1.0 / 240.0   # Simulation step duration (s)
GRAVITY = (0.0, 0.0, -9.81)


# =========================
# MOTOR CONFIGURATION
# =========================

MOTOR_FORCE = 12.0        # Maximum motor torque (N*m)
MOTOR_MAX_VELOCITY = 2.0  # Motor velocity limit (rad/s)


# =========================
# TRAJECTORY CONFIGURATION
# =========================

MIN_TRAJECTORY_DURATION = 2.0  # Minimum planned duration (s)
PLANNED_SPEED_LIMIT = 1.5      # Peak planned joint speed (rad/s)


# =========================
# FILE PATHS
# =========================

REPO_ROOT = Path(__file__).resolve().parents[2]

ARM_URDF = REPO_ROOT / "models" / "2_dof_arm.urdf"
