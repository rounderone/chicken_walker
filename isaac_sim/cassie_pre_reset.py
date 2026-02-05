# Set Cassie joints BEFORE world reset
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

# Clear
send_command("execute_script", {"code": '''
from omni.isaac.core.world import World
try:
    World.clear_instance()
except:
    pass
import omni.usd
omni.usd.get_context().new_stage()
import omni.kit.app
omni.kit.app.get_app().update()
'''})
time.sleep(2)

script = '''
import numpy as np
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf

from omni.isaac.core.world import World
from omni.isaac.core.articulations import Articulation

print("Creating World (no reset yet)...")
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
            op.Set(Gf.Vec3d(0, 0, 0.85))
            break

# Add articulation (but don't reset yet)
cassie = Articulation(prim_path="/World/Cassie", name="cassie_bot")
world.scene.add(cassie)

# Bent-knee positions
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

# Build position array (we need to guess the order before initialization)
# Standard order: hip_abd_L, hip_abd_R, hip_rot_L, hip_rot_R, hip_flex_L, hip_flex_R,
#                 knee_L, knee_R, shin_L, shin_R, ankle_L, ankle_R, toe_L, toe_R
dof_order = [
    "hip_abduction_left", "hip_abduction_right",
    "hip_rotation_left", "hip_rotation_right",
    "hip_flexion_left", "hip_flexion_right",
    "knee_joint_left", "knee_joint_right",
    "knee_to_shin_left", "knee_to_shin_right",
    "ankle_joint_left", "ankle_joint_right",
    "toe_joint_left", "toe_joint_right"
]

default_positions = np.array([joint_positions.get(name, 0.0) for name in dof_order])
default_velocities = np.zeros(14)

print("Setting default state BEFORE reset...")
print(f"Positions: hip_flex=34deg, knee=-57deg, ankle=57deg, toe=-57deg")

# Now reset - this will use the default state
print("Resetting world...")
world.reset()

# After reset, the articulation should be initialized
# Now set the joint positions
if cassie.dof_names is not None:
    actual_order = list(cassie.dof_names)
    positions = np.array([joint_positions.get(name, 0.0) for name in actual_order])
    cassie.set_joint_positions(positions)
    cassie.set_joints_default_state(positions=positions)
    print(f"Set {len(positions)} joint positions")

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util

viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(2.5, -2.5, 1.5), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.6), True)

print("\\n=== Press PLAY ===")
'''

print("Setting up Cassie with pre-set joints...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken - check Isaac Sim and press PLAY!")
