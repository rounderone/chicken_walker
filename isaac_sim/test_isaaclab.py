"""
Test Isaac Lab installation
Run with: D:\Projects\ATST\venv_isaaclab\Scripts\python.exe D:\Projects\ATST\isaac_sim\test_isaaclab.py
"""
import os
import sys

# Accept EULA automatically
os.environ["ACCEPT_EULA"] = "Y"

print("=" * 60)
print("Testing Isaac Lab Installation")
print("=" * 60)
sys.stdout.flush()

from isaacsim import SimulationApp

# Start simulation
print("[1] Starting SimulationApp...")
sys.stdout.flush()
simulation_app = SimulationApp({"headless": False})

print("[2] Importing Isaac Lab modules...")
sys.stdout.flush()

import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation

print("[3] Creating simulation context...")
sys.stdout.flush()
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.01))

print("[4] Adding ground plane...")
sys.stdout.flush()
cfg = sim_utils.GroundPlaneCfg()
cfg.func("/World/defaultGroundPlane", cfg)

print("[5] Adding dome light...")
sys.stdout.flush()
light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
light_cfg.func("/World/Light", light_cfg)

print("[6] Setting camera...")
sys.stdout.flush()
sim.set_camera_view(eye=[5.0, 5.0, 5.0], target=[0.0, 0.0, 0.0])

print("[7] Initializing simulation...")
sys.stdout.flush()
sim.reset()

print("=" * 60)
print("SUCCESS! Isaac Lab is working!")
print("Close the window when done.")
print("=" * 60)
sys.stdout.flush()

# Run simulation loop
while simulation_app.is_running():
    sim.step()

simulation_app.close()
