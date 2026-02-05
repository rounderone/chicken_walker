# Complete Cassie setup - fresh scene with bent knees
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

# Step 1: Create fresh stage
script1 = '''
import omni.usd
print("Creating fresh stage...")
omni.usd.get_context().new_stage()
import omni.kit.app
omni.kit.app.get_app().update()
print("Done")
'''

print("Step 1: Fresh stage...")
result = send_command("execute_script", {"code": script1})
time.sleep(2)

# Step 2: Setup scene and load Cassie with bent knee settings
script2 = '''
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf, UsdLux

stage = omni.usd.get_context().get_stage()

# World and ground
UsdGeom.Xform.Define(stage, "/World")
ground = UsdGeom.Cube.Define(stage, "/World/Ground")
ground.CreateSizeAttr(10)
ground.CreateDisplayColorAttr([(0.3, 0.3, 0.3)])
gprim = stage.GetPrimAtPath("/World/Ground")
gxf = UsdGeom.Xformable(gprim)
gxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, -5))
UsdPhysics.CollisionAPI.Apply(gprim)

# Physics
physicsScene = UsdPhysics.Scene.Define(stage, "/physicsScene")
physicsScene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physicsScene.CreateGravityMagnitudeAttr(9.81)

# Light
light = UsdLux.DistantLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3000)

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

# Position Cassie - feet at ground
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    xf = UsdGeom.Xformable(cassie)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 1.15))
            break
    print("Cassie at Z=1.15")

# Configure joints with bent-knee targets
joint_targets = {
    "hip_flexion": 5,
    "knee_joint": -15,
    "knee_to_shin": -10,
    "ankle_joint": 40,      # Main reverse-knee bend
    "toe_joint": -20,
}

joint_stiffness = {
    "hip": 600,
    "knee": 400,
    "ankle": 500,
    "toe": 300,
}

joints_set = 0
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/Cassie" not in path or "Joint" not in prim.GetTypeName():
        continue

    name = prim.GetName().lower()

    if prim.HasAPI(UsdPhysics.DriveAPI):
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if drive:
            target = 0.0
            for key, angle in joint_targets.items():
                if key in name:
                    target = angle
                    break

            stiff = 500
            for key, s in joint_stiffness.items():
                if key in name:
                    stiff = s
                    break

            drive.CreateStiffnessAttr(stiff)
            drive.CreateDampingAttr(stiff * 0.2)
            drive.CreateTargetPositionAttr(target)
            joints_set += 1

print(f"Configured {joints_set} joints with bent-knee stance")

# Camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util

viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(2.5, -2.5, 1.5), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 0.8), True)
    print("Camera set")

print("")
print("=== READY ===")
print("Press PLAY - Cassie should stand with bent knees!")
'''

print("Step 2: Loading Cassie with bent-knee setup...")
result = send_command("execute_script", {"code": script2})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
