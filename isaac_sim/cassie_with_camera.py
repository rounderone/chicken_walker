# Reload Cassie and set up camera to view it
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
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf, Sdf

stage = omni.usd.get_context().get_stage()

# Remove old Cassie
if stage.GetPrimAtPath("/World/Cassie"):
    stage.RemovePrim("/World/Cassie")

print("Loading Cassie...")

from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path

assets_root = get_assets_root_path()
cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"
add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")

# Position Cassie at Z=1.2
cassie = stage.GetPrimAtPath("/World/Cassie")
cassie_xf = UsdGeom.Xformable(cassie)
for op in cassie_xf.GetOrderedXformOps():
    if "translate" in op.GetOpName():
        op.Set(Gf.Vec3d(0, 0, 1.2))
        print("Positioned at Z=1.2")
        break

# Increase joint stiffness
STIFFNESS_MULT = 20.0
DAMPING_MULT = 50.0

joints_modified = 0
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/Cassie" in path and "Joint" in prim.GetTypeName():
        if prim.HasAPI(UsdPhysics.DriveAPI):
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drive:
                stiff_attr = drive.GetStiffnessAttr()
                damp_attr = drive.GetDampingAttr()

                if stiff_attr and stiff_attr.Get():
                    stiff_attr.Set(stiff_attr.Get() * STIFFNESS_MULT)

                if damp_attr and damp_attr.Get():
                    damp_attr.Set(damp_attr.Get() * DAMPING_MULT)

                joints_modified += 1

print(f"Modified {joints_modified} joints")

# Create or update a camera that looks at Cassie
camera_path = "/World/ViewCamera"
if not stage.GetPrimAtPath(camera_path):
    camera = UsdGeom.Camera.Define(stage, camera_path)
else:
    camera = UsdGeom.Camera.Get(stage, camera_path)

cam_prim = stage.GetPrimAtPath(camera_path)
cam_xf = UsdGeom.Xformable(cam_prim)
cam_xf.ClearXformOpOrder()

# Position camera to look at Cassie from front-right
cam_xf.AddTranslateOp().Set(Gf.Vec3d(3.0, -3.0, 1.5))

# Rotate to look at origin - camera looks down -Z by default
# We need to rotate to face the origin
import math
cam_xf.AddRotateXYZOp().Set(Gf.Vec3f(70, 0, 135))

print(f"Created camera at {camera_path}")
print("")
print("In Isaac Sim: Right-click ViewCamera in Stage > Set as Active Camera")
print("Then press PLAY to see if Cassie stands!")
'''

print("Setting up Cassie with camera...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken (may still be from old camera)")
