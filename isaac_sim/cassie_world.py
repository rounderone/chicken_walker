# Cassie using proper World API for physics
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

# Clear first
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
from pxr import Gf

from omni.isaac.core.world import World
from omni.isaac.core.articulations import Articulation

print("Creating World with physics...")
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Load Cassie
print("Loading Cassie...")
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path

assets_root = get_assets_root_path()
cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/configuration/cassie_physics.usd"
add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")

# Position higher - raise to 1.05m so feet clear ground
from pxr import UsdGeom, UsdPhysics
stage = omni.usd.get_context().get_stage()
cassie_prim = stage.GetPrimAtPath("/World/Cassie")
if cassie_prim:
    xf = UsdGeom.Xformable(cassie_prim)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 0.9))  # Isaac Lab uses 0.9
            print("Cassie at Z=0.9 (Isaac Lab default)")
            break

# Configure joint drives BEFORE physics init
import math
print("\\nConfiguring joint drives...")

# Using Isaac Lab values (converted from radians to degrees)
# Isaac Lab uses: hip_flexion=1.0rad, thigh_joint=-1.8rad, ankle=1.57rad, toe=-1.57rad
# But our USD has "knee_joint" not "thigh_joint"
joint_targets_rad = {
    "hip_abduction_left": 0.1,
    "hip_abduction_right": -0.1,
    "hip_flexion_left": 1.2,       # 69 deg forward - more lean
    "hip_flexion_right": 1.2,
    "knee_joint_left": -2.5,       # -143 deg - near limit!
    "knee_joint_right": -2.5,
    "ankle_joint_left": 2.0,       # 115 deg
    "ankle_joint_right": 2.0,
    "toe_joint_left": -1.57,       # -90 deg
    "toe_joint_right": -1.57,
}

# Much higher stiffness for static standing (not RL control)
STIFFNESS_MAP = {
    "hip_abduction": 1000.0,
    "hip_rotation": 1000.0,
    "hip_flexion": 2000.0,
    "knee_joint": 2000.0,
    "ankle_joint": 2000.0,
    "toe_joint": 500.0,
}
DAMPING_MAP = {
    "hip_abduction": 100.0,
    "hip_rotation": 100.0,
    "hip_flexion": 200.0,
    "knee_joint": 200.0,
    "ankle_joint": 200.0,
    "toe_joint": 50.0,
}

for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/Cassie" not in path:
        continue

    prim_type = prim.GetTypeName()
    if "Joint" not in prim_type:
        continue

    name = prim.GetName()

    # Apply DriveAPI if not present
    if not prim.HasAPI(UsdPhysics.DriveAPI):
        UsdPhysics.DriveAPI.Apply(prim, "angular")

    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if drive:
        drive.CreateTypeAttr("force")

        # Get stiffness/damping based on joint type
        stiff = 100.0
        damp = 3.0
        for key in STIFFNESS_MAP:
            if key in name:
                stiff = STIFFNESS_MAP[key]
                damp = DAMPING_MAP[key]
                break

        drive.CreateStiffnessAttr(stiff)
        drive.CreateDampingAttr(damp)
        drive.CreateMaxForceAttr(1000.0)  # Increase max force!

        target_rad = joint_targets_rad.get(name, 0.0)
        drive.CreateTargetPositionAttr(target_rad)

        if abs(target_rad) > 0.01:
            print(f"  {name}: {math.degrees(target_rad):.1f} deg, stiff={stiff}")

print("Joint drives configured!")

# Create articulation wrapper
print("Creating Articulation...")
cassie = Articulation(prim_path="/World/Cassie", name="cassie_robot")
world.scene.add(cassie)

# Reset world - THIS initializes physics properly
print("Resetting world (initializes physics)...")
world.reset()

# IMMEDIATELY STOP physics after reset
import omni.timeline
timeline = omni.timeline.get_timeline_interface()
timeline.stop()
print("Physics STOPPED - setting positions...")

# Now cassie should be initialized
print(f"DOFs: {cassie.num_dof}")
print(f"Joint names: {list(cassie.dof_names)}")

# Set joint positions to MATCH drive targets (more bent)
joint_positions = {
    "hip_abduction_left": 0.1,
    "hip_abduction_right": -0.1,
    "hip_rotation_left": 0.0,
    "hip_rotation_right": 0.0,
    "hip_flexion_left": 1.2,
    "hip_flexion_right": 1.2,
    "knee_joint_left": -2.5,
    "knee_joint_right": -2.5,
    "knee_to_shin_left": 0.0,
    "knee_to_shin_right": 0.0,
    "ankle_joint_left": 2.0,
    "ankle_joint_right": 2.0,
    "toe_joint_left": -1.57,
    "toe_joint_right": -1.57,
}

# Build position array in correct order
dof_names = list(cassie.dof_names)
positions = np.zeros(len(dof_names))
for i, name in enumerate(dof_names):
    if name in joint_positions:
        positions[i] = joint_positions[name]
        deg = math.degrees(joint_positions[name])
        if abs(deg) > 1:
            print(f"  {name}: {deg:.1f} deg")

# Set positions AND velocities to zero
cassie.set_joint_positions(positions)
cassie.set_joint_velocities(np.zeros(len(dof_names)))
cassie.set_joints_default_state(positions=positions, velocities=np.zeros(len(dof_names)))

print("\\nJoint positions set!")

# Reset to apply the default state
world.reset()

# Set positions through articulation
cassie.set_joint_positions(positions)
cassie.set_joint_velocities(np.zeros(len(dof_names)))

# Use DC interface to ensure positions are set
from omni.isaac.dynamic_control import _dynamic_control
dc = _dynamic_control.acquire_dynamic_control_interface()
art = dc.get_articulation("/World/Cassie")
if art:
    print("Setting via DC interface...")
    for i, name in enumerate(dof_names):
        dof = dc.find_articulation_dof(art, name)
        if dof:
            dc.set_dof_position(dof, positions[i])
            dc.set_dof_velocity(dof, 0.0)
    dc.wake_up_articulation(art)

# Check actual positions
actual_pos = cassie.get_joint_positions()
print("Positions after DC set:")
for i, name in enumerate(dof_names):
    if 'knee' in name or 'hip_flex' in name:
        print(f"  {name}: {math.degrees(actual_pos[i]):.1f} deg")

# Run physics steps and force visual updates
print("Running physics steps...")
import omni.kit.app
for i in range(30):
    world.step(render=True)
    omni.kit.app.get_app().update()

# Check positions after physics
actual_pos = cassie.get_joint_positions()
print("Positions after physics:")
for i, name in enumerate(dof_names):
    if 'knee' in name or 'hip_flex' in name:
        print(f"  {name}: {math.degrees(actual_pos[i]):.1f} deg")

# More visual updates
for _ in range(10):
    omni.kit.app.get_app().update()

timeline.stop()
print("Timeline stopped")

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util

viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(2.5, -2.5, 1.2), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.5), True)

print("\\n=== Press PLAY to run physics ===")
print("World.reset() has initialized physics")
'''

print("Setting up Cassie with World API...")
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
