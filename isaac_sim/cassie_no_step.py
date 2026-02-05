# Cassie setup WITHOUT stepping physics - just set pose
import socket
import json
import time

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

script = '''
import numpy as np
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf, UsdLux

from omni.isaac.core.world import World
from omni.isaac.core.articulations import Articulation

print("Creating World...")
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Load Cassie
print("Loading Cassie...")
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path

assets_root = get_assets_root_path()
cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"
add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")

# Position
stage = omni.usd.get_context().get_stage()
cassie_prim = stage.GetPrimAtPath("/World/Cassie")
if cassie_prim:
    xf = UsdGeom.Xformable(cassie_prim)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 0.9))
            break

# Create articulation
print("Creating Articulation...")
cassie = Articulation(prim_path="/World/Cassie", name="cassie")
world.scene.add(cassie)

# Reset (this initializes physics)
print("Resetting...")
world.reset()

# Set joint positions for crouched stance
# Using more moderate angles
joint_positions = {
    "hip_abduction_left": 0.05,
    "hip_abduction_right": -0.05,
    "hip_rotation_left": 0.0,
    "hip_rotation_right": 0.0,
    "hip_flexion_left": 0.6,
    "hip_flexion_right": 0.6,
    "knee_joint_left": -1.0,
    "knee_joint_right": -1.0,
    "knee_to_shin_left": 0.0,
    "knee_to_shin_right": 0.0,
    "ankle_joint_left": 1.0,
    "ankle_joint_right": 1.0,
    "toe_joint_left": -1.0,
    "toe_joint_right": -1.0,
}

dof_names = list(cassie.dof_names)
positions = np.zeros(len(dof_names))

print("\\nSetting positions:")
for i, name in enumerate(dof_names):
    if name in joint_positions:
        positions[i] = joint_positions[name]
        if abs(joint_positions[name]) > 0.01:
            print(f"  {name}: {np.degrees(joint_positions[name]):.1f} deg")

# Set positions
cassie.set_joint_positions(positions)
cassie.set_joints_default_state(positions=positions)

# DO NOT step physics - let user press play

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util

viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(2.5, -2.5, 1.5), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.6), True)

print("\\n=== Cassie in crouched pose ===")
print("Press PLAY in Isaac Sim!")
'''

print("Setting up Cassie without physics stepping...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
