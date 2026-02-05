# H1 Humanoid standalone demo - launches Isaac Sim directly
# Run with: D:\Projects\ATST\venv_isaaclab\Scripts\python.exe h1_standalone.py

import sys
print("Starting Isaac Sim...", flush=True)
sys.stdout.flush()

from omni.isaac.kit import SimulationApp

# Launch Isaac Sim with GUI
simulation_app = SimulationApp({"headless": False})

print("Isaac Sim started, loading modules...", flush=True)
sys.stdout.flush()

import numpy as np
import torch

from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction

print("Creating world...", flush=True)
sys.stdout.flush()

# Create world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Get assets path
assets_root = get_assets_root_path()
print(f"Assets root: {assets_root}", flush=True)

if assets_root is None:
    print("ERROR: Could not connect to Nucleus. Check your connection.", flush=True)
    simulation_app.close()
    sys.exit(1)

# H1 robot path
h1_usd = f"{assets_root}/Isaac/Robots/Unitree/H1/h1.usd"
print(f"Loading H1 from: {h1_usd}", flush=True)
sys.stdout.flush()

try:
    # Add H1 to scene
    from omni.isaac.core.robots import Robot
    h1 = world.scene.add(
        Robot(
            prim_path="/World/H1",
            usd_path=h1_usd,
            name="h1",
            position=np.array([0, 0, 1.05]),
        )
    )
    print("H1 loaded successfully!", flush=True)
except Exception as e:
    print(f"Failed to load H1: {e}", flush=True)
    print("Trying alternative paths...", flush=True)

    # Try alternative paths
    alt_paths = [
        f"{assets_root}/Isaac/Robots/Humanoid/unitree_h1/h1.usd",
        f"{assets_root}/Isaac/Robots/Unitree/h1/h1.usd",
    ]
    h1 = None
    for path in alt_paths:
        try:
            print(f"  Trying: {path}", flush=True)
            h1 = world.scene.add(
                Robot(
                    prim_path="/World/H1",
                    usd_path=path,
                    name="h1",
                    position=np.array([0, 0, 1.05]),
                )
            )
            print(f"  SUCCESS with: {path}", flush=True)
            break
        except:
            continue

    if h1 is None:
        print("Could not load H1 from any path", flush=True)

# Set camera view
world.scene.add_default_camera(
    position=np.array([3, -3, 2]),
    target=np.array([0, 0, 1])
)

# Reset world
print("Resetting world...", flush=True)
sys.stdout.flush()
world.reset()

# Get joint info
if h1 is not None:
    print(f"\nH1 DOFs: {h1.num_dof}", flush=True)
    print(f"Joint names: {h1.dof_names}", flush=True)

    # Get default joint positions
    default_pos = h1.get_joint_positions()
    print(f"Default positions: {default_pos}", flush=True)

print("\n=== H1 Humanoid Loaded ===", flush=True)
print("The robot will try to maintain its standing pose.", flush=True)
print("Close the window to exit.", flush=True)
sys.stdout.flush()

# Simple standing control loop
step_count = 0
while simulation_app.is_running():
    world.step(render=True)

    if h1 is not None and step_count % 2 == 0:
        # Command default positions to maintain stance
        h1.set_joint_position_targets(h1.get_joint_positions())

    step_count += 1

    if step_count % 500 == 0:
        if h1 is not None:
            pos = h1.get_world_pose()[0]
            print(f"Step {step_count}: H1 position = {pos}", flush=True)

simulation_app.close()
print("Done!", flush=True)
