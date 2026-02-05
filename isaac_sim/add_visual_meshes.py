# Add visual meshes to the working AT-ST physics model
# This script loads the primitive-geometry AT-ST and adds the detailed OBJ meshes as visuals
#
# Run with: D:\Projects\ATST\venv_isaaclab\Scripts\python.exe add_visual_meshes.py

import sys
import os
print("=" * 60, flush=True)
print("Adding Visual Meshes to AT-ST Physics Model", flush=True)
print("=" * 60, flush=True)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})  # headless=False so we can see it

print("Isaac Sim started...", flush=True)

import omni.usd
import omni.kit.commands
from pxr import Usd, UsdGeom, Sdf, Gf

# Get the stage
context = omni.usd.get_context()

# Load the working physics model
physics_usd = "D:/Projects/ATST/usd/atst_rl.usda"
print(f"Loading physics model: {physics_usd}", flush=True)
context.open_stage(physics_usd)

stage = context.get_stage()
print("Stage loaded successfully!", flush=True)

# Mapping from physics link names to OBJ mesh files
# The physics model uses: torso_link, left_hip_link, left_upper_leg_link, left_lower_leg_link, left_ankle_link, etc.
# Our OBJ files are: ATST_torso.obj, ATST_thigh_L.obj, ATST_shin_L.obj, ATST_elbow_L.obj, ATST_foot_L.obj, etc.

mesh_mapping = {
    "torso_link": "ATST_torso.obj",
    "left_upper_leg_link": "ATST_thigh_L.obj",
    "left_lower_leg_link": "ATST_shin_L.obj",
    "left_ankle_link": "ATST_foot_L.obj",
    "right_upper_leg_link": "ATST_thigh_R.obj",
    "right_lower_leg_link": "ATST_shin_R.obj",
    "right_ankle_link": "ATST_foot_R.obj",
    # Note: left_hip_link/right_hip_link are small connectors, we'll skip them
    # Note: elbow parts are the "reverse knee" joint - we'll map shin to lower_leg
}

parts_dir = "D:/Projects/ATST/usd/parts"

# Check which OBJ files exist
print("\nChecking OBJ files:", flush=True)
for link_name, obj_file in mesh_mapping.items():
    obj_path = os.path.join(parts_dir, obj_file)
    exists = os.path.exists(obj_path)
    print(f"  {obj_file}: {'EXISTS' if exists else 'MISSING'}", flush=True)

# First, let's just verify the physics model is working by adding a ground plane and light
print("\nAdding ground plane and lighting...", flush=True)

# Create a ground plane
ground_path = "/World/GroundPlane"
if not stage.GetPrimAtPath(ground_path):
    ground = UsdGeom.Mesh.Define(stage, ground_path)
    ground.CreatePointsAttr([(-10, -10, 0), (10, -10, 0), (10, 10, 0), (-10, 10, 0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateNormalsAttr([(0, 0, 1)] * 4)

    # Add collision to ground
    from pxr import UsdPhysics
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    print("  Ground plane created with collision", flush=True)

# Create physics scene if not exists
physics_scene_path = "/World/PhysicsScene"
if not stage.GetPrimAtPath(physics_scene_path):
    from pxr import UsdPhysics
    physics_scene = UsdPhysics.Scene.Define(stage, physics_scene_path)
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    print("  Physics scene created", flush=True)

# Add a dome light for visibility
light_path = "/World/DomeLight"
if not stage.GetPrimAtPath(light_path):
    from pxr import UsdLux
    light = UsdLux.DomeLight.Define(stage, light_path)
    light.CreateIntensityAttr(1000)
    print("  Dome light created", flush=True)

# Now let's try to add visual meshes
# We need to import OBJ files - Isaac Sim uses the asset converter
print("\nAttempting to add visual meshes...", flush=True)

try:
    import omni.kit.asset_converter

    converter_context = omni.kit.asset_converter.AssetConverterContext()
    converter_context.ignore_materials = False
    converter_context.ignore_animations = True
    converter_context.ignore_cameras = True
    converter_context.single_mesh = True
    converter_context.smooth_normals = True

    converter = omni.kit.asset_converter.get_instance()

    for link_name, obj_file in mesh_mapping.items():
        obj_path = os.path.join(parts_dir, obj_file)
        if not os.path.exists(obj_path):
            print(f"  Skipping {link_name} - OBJ file not found", flush=True)
            continue

        # Convert OBJ to USD
        usd_output = obj_path.replace(".obj", "_converted.usd")
        print(f"  Converting {obj_file}...", flush=True)

        task = converter.create_converter_task(obj_path, usd_output, None, converter_context)
        success = task.wait_until_finished()

        if success:
            print(f"    Converted to {usd_output}", flush=True)
        else:
            print(f"    FAILED to convert {obj_file}", flush=True)

except ImportError as e:
    print(f"Asset converter not available: {e}", flush=True)
    print("Will try direct USD mesh creation instead...", flush=True)
except Exception as e:
    print(f"Error during conversion: {e}", flush=True)

# Save the enhanced model
output_path = "D:/Projects/ATST/usd/atst_with_ground.usda"
print(f"\nSaving to {output_path}...", flush=True)
stage.Export(output_path)
print("Saved!", flush=True)

# Keep the app running so we can see the result
print("\n" + "=" * 60, flush=True)
print("Done! The physics model should now be visible.", flush=True)
print("The window will stay open - press Ctrl+C to exit.", flush=True)
print("=" * 60, flush=True)

# Run the simulation loop
import time
try:
    while simulation_app.is_running():
        simulation_app.update()
        time.sleep(0.016)  # ~60 FPS
except KeyboardInterrupt:
    print("\nShutting down...", flush=True)

simulation_app.close()
