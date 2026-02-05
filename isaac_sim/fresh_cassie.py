# Fresh scene with just Cassie
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

# First create a new stage
script1 = '''
import omni.usd

# Create new stage
print("Creating new stage...")
omni.usd.get_context().new_stage()

import omni.kit.app
omni.kit.app.get_app().update()

print("New stage created")
'''

print("Creating fresh stage...")
result = send_command("execute_script", {"code": script1})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)

time.sleep(2)

# Now set up the scene
script2 = '''
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf, UsdLux

stage = omni.usd.get_context().get_stage()

# Create World xform
UsdGeom.Xform.Define(stage, "/World")

# Create ground
ground = UsdGeom.Cube.Define(stage, "/World/Ground")
ground.CreateSizeAttr(10)
ground.CreateDisplayColorAttr([(0.3, 0.3, 0.3)])
gprim = stage.GetPrimAtPath("/World/Ground")
gxf = UsdGeom.Xformable(gprim)
gxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, -5))
UsdPhysics.CollisionAPI.Apply(gprim)

# Create physics scene
physicsScene = UsdPhysics.Scene.Define(stage, "/physicsScene")
physicsScene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physicsScene.CreateGravityMagnitudeAttr(9.81)

# Create light
light = UsdLux.DistantLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3000)

print("Scene setup complete")

# Now load Cassie
print("\\nLoading Cassie from Nucleus assets...")
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path

assets_root = get_assets_root_path()
print(f"Assets root: {assets_root}")

cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"
print(f"Loading: {cassie_usd}")

add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")

# Wait for assets to load
import omni.kit.app
for i in range(10):
    omni.kit.app.get_app().update()

# Position Cassie
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    xf = UsdGeom.Xformable(cassie)
    # Find existing translate op and modify it
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 1.15))  # Feet just touching ground
            print("Cassie positioned at Z=1.15 (feet at ground)")
            break

    # Count meshes
    mesh_count = 0
    for prim in stage.Traverse():
        if "/World/Cassie" in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh):
            mesh_count += 1
    print(f"Meshes loaded: {mesh_count}")

# Set up camera
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_util

viewport = vp_util.get_active_viewport()
if viewport:
    cam_path = viewport.get_active_camera()
    camera_state = ViewportCameraState(cam_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(3.0, -3.0, 2.0), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 1.0), True)
    print("Camera positioned")

print("\\nReady - press PLAY!")
'''

print("\nSetting up scene with Cassie...")
result = send_command("execute_script", {"code": script2})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(3)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
