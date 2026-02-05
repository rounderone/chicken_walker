# Cassie with ACTIVE joint control during simulation
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
import math
from pxr import Gf, UsdPhysics, UsdGeom

from omni.isaac.core.world import World
from omni.isaac.core.articulations import Articulation

print("Creating World...")
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Load Cassie
print("Loading Cassie...")
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path
import omni.usd
from pxr import UsdGeom

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
            op.Set(Gf.Vec3d(0, 0, 1.2))  # Higher - feet above ground
            break

# Update stage to ensure Cassie is loaded
import omni.kit.app
for _ in range(10):
    omni.kit.app.get_app().update()

# Set high stiffness/damping on ALL joints
print("Setting joint drives...")
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/Cassie" not in path or "Joint" not in prim.GetTypeName():
        continue

    name = prim.GetName()

    if not prim.HasAPI(UsdPhysics.DriveAPI):
        UsdPhysics.DriveAPI.Apply(prim, "angular")

    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if drive:
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(50000.0)  # Extremely high
        drive.CreateDampingAttr(5000.0)
        drive.CreateMaxForceAttr(100000.0)  # Massive force limit

        # Set drive targets for standing pose
        target_rad = {
            "hip_flexion_left": 1.0,
            "hip_flexion_right": 1.0,
            "knee_joint_left": -1.8,
            "knee_joint_right": -1.8,
            "ankle_joint_left": 1.57,
            "ankle_joint_right": 1.57,
            "toe_joint_left": -1.57,
            "toe_joint_right": -1.57,
        }.get(name, 0.0)
        drive.CreateTargetPositionAttr(target_rad)

# Create articulation
cassie = Articulation(prim_path="/World/Cassie", name="cassie")
world.scene.add(cassie)

# Reset to initialize
print("Initializing...")
world.reset()

# Get joint info
dof_names = list(cassie.dof_names)
print(f"DOFs: {len(dof_names)}")

# Isaac Lab standing pose
target_positions = {
    "hip_abduction_left": 0.1,
    "hip_abduction_right": -0.1,
    "hip_rotation_left": 0.0,
    "hip_rotation_right": 0.0,
    "hip_flexion_left": 1.0,
    "hip_flexion_right": 1.0,
    "knee_joint_left": -1.8,
    "knee_joint_right": -1.8,
    "knee_to_shin_left": 0.0,
    "knee_to_shin_right": 0.0,
    "ankle_joint_left": 1.57,
    "ankle_joint_right": 1.57,
    "toe_joint_left": -1.57,
    "toe_joint_right": -1.57,
}

# Build position array
positions = np.array([target_positions.get(name, 0.0) for name in dof_names])

# Set initial state
cassie.set_joint_positions(positions)
cassie.set_joint_velocities(np.zeros(len(dof_names)))

# Run simulation - let joint DRIVES do the work (not set_joint_positions)
print("Running with joint DRIVES (PD control)...")
print("Cassie starting at Z=1.2 (feet should be above ground)")
print("NOT calling set_joint_positions - letting drives control")
for step in range(600):  # 10 seconds
    # DON'T set joint positions - let the drives work
    world.step(render=True)

    if step % 60 == 0:
        # Check pelvis height to see if fallen
        pelvis_prim = stage.GetPrimAtPath("/World/Cassie/pelvis")
        if pelvis_prim:
            xf = UsdGeom.Xformable(pelvis_prim)
            world_tf = xf.ComputeLocalToWorldTransform(0)
            pelvis_z = world_tf.GetRow(3)[2]
            actual = cassie.get_joint_positions()
            knee_pos = actual[dof_names.index("knee_joint_left")]
            print(f"  Step {step}: pelvis_z={pelvis_z:.2f}, knee={math.degrees(knee_pos):.1f} deg")

# DON'T stop - keep simulation running
print("Simulation still running - watch in Isaac Sim!")
print("(Press STOP in Isaac Sim when done)")

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util
viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(2.5, -2.5, 1.2), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.6), True)

print("")
print("=== Simulation complete ===")
print("Check if Cassie maintained pose!")
'''

print("Running Cassie with active control...")
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
