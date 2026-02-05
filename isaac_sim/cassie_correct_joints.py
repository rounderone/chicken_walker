# Cassie with CORRECT joint names and direct position setting
import socket
import json
import time
import numpy as np

def send_command(command_type, params=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(120)
    try:
        sock.connect(("localhost", 8766))
        command = {"type": command_type, "params": params or {}}
        sock.sendall(json.dumps(command).encode('utf-8'))
        response = sock.recv(65536)
        return json.loads(response.decode('utf-8'))
    finally:
        sock.close()

# Fresh stage
print("Creating fresh stage...")
send_command("execute_script", {"code": '''
import omni.usd
omni.usd.get_context().new_stage()
import omni.kit.app
omni.kit.app.get_app().update()
'''})
time.sleep(2)

# Setup scene and load Cassie
script1 = '''
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf, UsdLux

stage = omni.usd.get_context().get_stage()

UsdGeom.Xform.Define(stage, "/World")
ground = UsdGeom.Cube.Define(stage, "/World/Ground")
ground.CreateSizeAttr(20)
ground.CreateDisplayColorAttr([(0.2, 0.2, 0.2)])
gprim = stage.GetPrimAtPath("/World/Ground")
UsdGeom.Xformable(gprim).AddTranslateOp().Set(Gf.Vec3d(0, 0, -10))
UsdPhysics.CollisionAPI.Apply(gprim)

physicsScene = UsdPhysics.Scene.Define(stage, "/physicsScene")
physicsScene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physicsScene.CreateGravityMagnitudeAttr(9.81)

UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000)

print("Loading Cassie...")
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path

assets_root = get_assets_root_path()
cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"
add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")

import omni.kit.app
for i in range(10):
    omni.kit.app.get_app().update()

cassie_prim = stage.GetPrimAtPath("/World/Cassie")
if cassie_prim:
    xf = UsdGeom.Xformable(cassie_prim)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 0.9))
            break

print("Scene ready")
'''

print("Setting up scene...")
result = send_command("execute_script", {"code": script1})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)

time.sleep(1)

# Now use Articulation with CORRECT joint names
script2 = '''
import numpy as np
from omni.isaac.core.articulations import Articulation

print("Creating Articulation...")
cassie = Articulation(prim_path="/World/Cassie")
cassie.initialize()

print(f"DOF names: {cassie.dof_names}")
print(f"Num DOFs: {cassie.num_dof}")

# CORRECT joint positions for Cassie (in radians)
# Based on actual joint names from inspection
# Joint order: hip_abd_L, hip_abd_R, hip_rot_L, hip_rot_R, hip_flex_L, hip_flex_R,
#              knee_L, knee_R, knee_shin_L, knee_shin_R, ankle_L, ankle_R, toe_L, toe_R

joint_positions = {
    # Hips
    "hip_abduction_left": 0.1,
    "hip_abduction_right": -0.1,
    "hip_rotation_left": 0.0,
    "hip_rotation_right": 0.0,
    "hip_flexion_left": 1.0,       # Forward lean
    "hip_flexion_right": 1.0,

    # KNEES - this is the key for the reverse-knee bend!
    "knee_joint_left": -1.5,       # Bend knee back
    "knee_joint_right": -1.5,
    "knee_to_shin_left": 0.0,      # Part of knee mechanism
    "knee_to_shin_right": 0.0,

    # Ankles - main reverse-knee angle
    "ankle_joint_left": 1.5,       # ~90 degrees
    "ankle_joint_right": 1.5,

    # Toes - keep feet flat
    "toe_joint_left": -1.5,        # Compensate for ankle
    "toe_joint_right": -1.5,
}

# Build position array in correct order
dof_names = list(cassie.dof_names)
positions = np.zeros(len(dof_names))

print("\\nSetting joint positions:")
for i, name in enumerate(dof_names):
    if name in joint_positions:
        positions[i] = joint_positions[name]
        print(f"  {name}: {joint_positions[name]:.2f} rad ({np.degrees(joint_positions[name]):.1f} deg)")

# Set joint positions DIRECTLY
print("\\nApplying positions...")
cassie.set_joint_positions(positions)

# Also set as default state so resets work
cassie.set_joints_default_state(positions=positions)

print("\\nJoint positions set!")
print("\\n=== Press PLAY ===")
'''

print("\nSetting up Articulation with correct joints...")
result = send_command("execute_script", {"code": script2})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

# Camera
time.sleep(1)
send_command("execute_script", {"code": '''
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util
from pxr import Gf

viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(2.0, -2.0, 1.0), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.5), True)
'''})

time.sleep(1)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
