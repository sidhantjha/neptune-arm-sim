
import pybullet as p
import pybullet_data

from src.common.config import TIME_STEP, GRAVITY


# ==========================================
# INITIALIZE THE PHYSICS SIMULATION
# ==========================================

def start_simulation(gui=True):

    # Choose graphical or headless simulation
    mode = p.GUI if gui else p.DIRECT

    client = p.connect(mode)

    if client < 0:
        raise RuntimeError("Could not connect to PyBullet")

    # Configure the physics environment
    p.setAdditionalSearchPath(
        pybullet_data.getDataPath(),
        physicsClientId=client
    )

    p.setGravity(
        *GRAVITY,
        physicsClientId=client
    )

    p.setTimeStep(
        TIME_STEP,
        physicsClientId=client
    )

    # Physics advances only when we request a step
    p.setRealTimeSimulation(
        0,
        physicsClientId=client
    )

    # Load the ground plane
    p.loadURDF(
        "plane.urdf",
        physicsClientId=client
    )

    return client


# ==========================================
# ADVANCE THE PHYSICS
# ==========================================

def step_simulation(client):

    p.stepSimulation(
        physicsClientId=client
    )


# ==========================================
# DISCONNECT THE SIMULATION
# ==========================================

def stop_simulation(client):

    if p.isConnected(client):
        p.disconnect(
            physicsClientId=client
        )


# ==========================================
# MANUAL VERIFICATION
# ==========================================

if __name__ == "__main__":

    client = start_simulation(gui=False)

    try:
        print("Connected:", p.isConnected(client))
        print(
            "Bodies loaded:",
            p.getNumBodies(physicsClientId=client)
        )

        for _ in range(240):
            step_simulation(client)

        print("Advanced 240 physics steps successfully")

    finally:
        stop_simulation(client)

    print("Simulation closed successfully")
