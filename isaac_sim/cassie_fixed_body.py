# Cassie with FIXED pelvis - legs can move but body won't fall
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

print("Setting up scene...")
stage = omni.usd.get_context().get_stage()

# Basic scene
UsdGeom.Xform.Define(stage, "/World")
ground = UsdGeom.Mesh.Define(stage, "/World/Ground")
ground.CreatePointsAttr([(-10, -10, 0), (10, -10, 0), (10, 10, 0), (-10, 10, 0)])
ground.CreateFaceVertexCountsAttr([4])
ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath("/World/Ground"))

physicsScene = UsdPhysics.Scene.Define(stage, "/physicsScene")
physicsScene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physicsScene.CreateGravityMagnitudeAttr(9.81)

# Light
from pxr import UsdLux
UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(1500)

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

# Position higher
cassie_prim = stage.GetPrimAtPath("/World/Cassie")
if cassie_prim:
    xf = UsdGeom.Xformable(cassie_prim)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 1.0))  # Higher up
            break

# Make pelvis KINEMATIC (fixed in space)
print("Making pelvis kinematic...")
pelvis = stage.GetPrimAtPath("/World/Cassie/pelvis")
if pelvis:
    # Apply or get the RigidBodyAPI
    rb = UsdPhysics.RigidBodyAPI.Apply(pelvis) if not pelvis.HasAPI(UsdPhysics.RigidBodyAPI) else UsdPhysics.RigidBodyAPI(pelvis)
    rb.CreateKinematicEnabledAttr(True)
    print("Pelvis is now kinematic (won't fall)")

# Set joint drive targets for bent-knee pose
print("\\nSetting joint targets for bent-knee stance...")

joint_targets = {
    "hip_flexion_left": 45,      # Forward lean
    "hip_flexion_right": 45,
    "knee_joint_left": -60,      # Knee bend
    "knee_joint_right": -60,
    "ankle_joint_left": 60,      # Reverse knee
    "ankle_joint_right": 60,
    "toe_joint_left": -60,       # Flat feet
    "toe_joint_right": -60,
}

# Higher stiffness to reach targets quickly
STIFFNESS = 500
DAMPING = 50

joints_set = 0
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/Cassie" not in path or "Joint" not in prim.GetTypeName():
        continue

    name = prim.GetName()

    if prim.HasAPI(UsdPhysics.DriveAPI):
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if drive:
            drive.CreateStiffnessAttr(STIFFNESS)
            drive.CreateDampingAttr(DAMPING)

            target = joint_targets.get(name, 0)
            drive.CreateTargetPositionAttr(target)
            joints_set += 1

            if target != 0:
                print(f"  {name}: target={target} deg")

print(f"\\nConfigured {joints_set} joints")

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util

viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(2.5, -2.5, 1.5), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.8), True)

print("\\n=== PELVIS FIXED - Press PLAY ===")
print("Body won't fall, legs will move to bent position!")
'''

print("Setting up Cassie with fixed pelvis...")
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
