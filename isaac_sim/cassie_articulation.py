# Cassie using omni.isaac.core Articulation with direct joint state control
import socket
import json
import time
import math

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

# Basic scene
UsdGeom.Xform.Define(stage, "/World")

# Ground plane with collision
ground = UsdGeom.Cube.Define(stage, "/World/Ground")
ground.CreateSizeAttr(20)
ground.CreateDisplayColorAttr([(0.2, 0.2, 0.2)])
gprim = stage.GetPrimAtPath("/World/Ground")
UsdGeom.Xformable(gprim).AddTranslateOp().Set(Gf.Vec3d(0, 0, -10))
UsdPhysics.CollisionAPI.Apply(gprim)

# Physics scene
physicsScene = UsdPhysics.Scene.Define(stage, "/physicsScene")
physicsScene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physicsScene.CreateGravityMagnitudeAttr(9.81)

# Light
UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000)

# Load Cassie
print("Loading Cassie...")
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path

assets_root = get_assets_root_path()
cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"
add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")

import omni.kit.app
for i in range(10):
    omni.kit.app.get_app().update()

# Set initial position
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

# Now use Articulation to set joint states
script2 = '''
import math
import numpy as np

# Use omni.isaac.core Articulation
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.world import World

print("Creating Articulation wrapper...")

# We need a World to use Articulation properly
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Create Articulation for Cassie
cassie = Articulation(prim_path="/World/Cassie", name="cassie")
world.scene.add(cassie)

# Initialize the world (this starts physics)
world.reset()

print(f"Cassie DOF names: {cassie.dof_names}")
print(f"Num DOFs: {cassie.num_dof}")

# Isaac Lab default joint positions (in radians)
# From CASSIE_CFG init_state
default_positions = {
    "hip_abduction_left": 0.1,
    "hip_rotation_left": 0.0,
    "hip_flexion_left": 1.0,
    "knee_joint_left": -1.8,      # Deep knee bend
    "knee_to_shin_left": 0.0,
    "ankle_joint_left": 1.57,     # 90 degrees
    "toe_joint_left": -1.57,      # -90 degrees (flat foot)
    "hip_abduction_right": -0.1,
    "hip_rotation_right": 0.0,
    "hip_flexion_right": 1.0,
    "knee_joint_right": -1.8,
    "knee_to_shin_right": 0.0,
    "ankle_joint_right": 1.57,
    "toe_joint_right": -1.57,
}

# Build joint position array in correct order
joint_positions = []
for name in cassie.dof_names:
    if name in default_positions:
        joint_positions.append(default_positions[name])
    else:
        joint_positions.append(0.0)
        print(f"  Unknown joint: {name}")

joint_positions = np.array(joint_positions)
print(f"\\nSetting joint positions: {joint_positions}")

# Set joint positions directly
cassie.set_joint_positions(joint_positions)

# Also set as targets for the drives
cassie.set_joint_position_targets(joint_positions)

print("\\nJoint states set!")
print("Press PLAY - Cassie should start in crouched position!")
'''

print("\nSetting up Articulation and joint states...")
result = send_command("execute_script", {"code": script2})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(2)

# Camera
send_command("execute_script", {"code": '''
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util
from pxr import Gf

viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(2.5, -2.5, 1.5), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.7), True)
'''})

time.sleep(1)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
