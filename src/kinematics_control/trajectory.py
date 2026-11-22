# Smooth joint-space trajectory generation
def quintic_position(q_start, q_goal, t, duration):
    # duration is how long we want motion to take
    # s controls how far along the motion we are
    # t is elapsed time
    if duration <= 0:
        raise ValueError("DURATION MUST BE POSTIIVE")

    # Normalize time [0,1]
    u = max(0.0, min(1.0,t/duration))
    # Quintic Time Scaling Laws
    s = 10*u**3 - 15*u**4 + 6*u**5

    # Interpolate from starting to final joint angle
    q = q_start + s * (q_goal - q_start)
    return q


# Trajectory generation for BOTH joints
def quintic_joint_targets(q_start, q_goal, t, duration):
    # Inputs: q_start = (shoulder_start, elbow_start)
    shoulder_target = quintic_position(
        q_start[0],
        q_goal[0],
        t,
        duration
    )

    elbow_target = quintic_position(
        q_start[1],
        q_goal[1],
        t,
        duration
    )

    return shoulder_target, elbow_target








# Run this example only when trajectory.py is executed directly
if __name__ == "__main__":
    import math

    # Move one joint from 0 to 90 degrees
    q_start = math.radians(0)
    q_goal = math.radians(90)

    # Complete the movement in 2 seconds
    duration = 2.0

    # Sample the trajectory at different times
    for t in [0.0, 0.5, 1.0, 1.5, 2.0]:

        q = quintic_position(q_start, q_goal, t, duration)

        print(
            f"Time: {t:.1f} s | "
            f"Angle: {math.degrees(q):.2f} deg"
        )

