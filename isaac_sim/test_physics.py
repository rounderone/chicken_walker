"""
Test physics simulation directly in Python
Run with: C:\IS\python.bat D:\Projects\ATST\isaac_sim\test_physics.py
"""
print("=" * 60)
print("PHYSICS TEST - Running simulation in Python")
print("=" * 60)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni
import time
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
from omni.isaac.core import World

print("\n[1] Creating world and scene...")
world = World()

# Add ground plane using Isaac Sim's built-in
from omni.isaac.core.objects import GroundPlane
ground = GroundPlane("/World/Ground", z_position=-2)
print("    Ground plane at Z=-2")

# Add a simple cube that should fall - using numpy array for color
from omni.isaac.core.objects import DynamicCuboid
cube = DynamicCuboid(
    prim_path="/World/TestCube",
    name="test_cube",
    position=np.array([5, 0, 10]),  # 10 meters up
    size=1.0,
    color=np.array([1.0, 0.0, 0.0])  # Red
)
print("    Test cube at Z=10")

# Load AT-ST
print("\n[2] Loading AT-ST...")
stage = omni.usd.get_context().get_stage()
atst_prim = stage.DefinePrim("/World/ATST", "Xform")
atst_prim.GetReferences().AddReference("D:/Projects/ATST/usd/atst.usdc")

# Position AT-ST
atst_xform = UsdGeom.Xformable(atst_prim)
atst_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 10))
print("    AT-ST at Z=10")

# Add rigid body to AT-ST
rb = UsdPhysics.RigidBodyAPI.Apply(atst_prim)
rb.CreateRigidBodyEnabledAttr().Set(True)
rb.CreateKinematicEnabledAttr().Set(False)

mass = UsdPhysics.MassAPI.Apply(atst_prim)
mass.CreateMassAttr().Set(1000.0)

UsdPhysics.CollisionAPI.Apply(atst_prim)
print("    Physics applied to AT-ST")

# Reset world
print("\n[3] Initializing physics...")
world.reset()

# Get initial positions
def get_z_position(prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if prim:
        xform = UsdGeom.Xformable(prim)
        transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return transform.ExtractTranslation()[2]
    return None

cube_z_start = get_z_position("/World/TestCube")
atst_z_start = get_z_position("/World/ATST")
print(f"    Cube start Z: {cube_z_start}")
print(f"    AT-ST start Z: {atst_z_start}")

# Run simulation
print("\n[4] Running simulation for 2 seconds...")
for i in range(120):  # 120 steps at 60Hz = 2 seconds
    world.step(render=False)

    if i % 30 == 0:  # Print every 0.5 seconds
        cube_z = get_z_position("/World/TestCube")
        atst_z = get_z_position("/World/ATST")
        print(f"    Step {i}: Cube Z={cube_z:.2f}, AT-ST Z={atst_z:.2f}")

# Final positions
cube_z_end = get_z_position("/World/TestCube")
atst_z_end = get_z_position("/World/ATST")

print("\n" + "=" * 60)
print("RESULTS:")
print("=" * 60)
print(f"Cube:  Z {cube_z_start:.2f} -> {cube_z_end:.2f} (moved {cube_z_start - cube_z_end:.2f})")
print(f"AT-ST: Z {atst_z_start:.2f} -> {atst_z_end:.2f} (moved {atst_z_start - atst_z_end:.2f})")

if cube_z_end < cube_z_start - 1:
    print("\nCube FELL - physics is working!")
else:
    print("\nCube did NOT fall - physics broken!")

if atst_z_end < atst_z_start - 1:
    print("AT-ST FELL - success!")
else:
    print("AT-ST did NOT fall - something blocking physics")

print("=" * 60)

simulation_app.close()
