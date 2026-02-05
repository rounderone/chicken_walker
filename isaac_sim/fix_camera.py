# Fix camera to look at Cassie
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
from pxr import UsdGeom, Gf
import math

stage = omni.usd.get_context().get_stage()

camera_path = "/World/ViewCamera"
cam_prim = stage.GetPrimAtPath(camera_path)

if cam_prim:
    cam_xf = UsdGeom.Xformable(cam_prim)
    cam_xf.ClearXformOpOrder()

    # Camera at position looking toward origin
    # Place camera at (2, 2, 2) looking at (0, 0, 0.5)
    cam_pos = Gf.Vec3d(2.5, 2.5, 1.5)
    target = Gf.Vec3d(0, 0, 0.8)  # Cassie center roughly

    # Calculate look-at rotation
    forward = target - cam_pos
    forward = forward.GetNormalized()

    # In USD, camera looks down -Z, so we need to rotate
    # Yaw = atan2(-forward.x, -forward.y) for looking in XY plane
    # We'll use a simpler approach - position and euler angles

    cam_xf.AddTranslateOp().Set(cam_pos)
    # Rotate: first around Z to face origin, then tilt down
    # Camera at (2.5, 2.5) needs to rotate ~225 degrees around Z to face origin
    # Then tilt down slightly to look at Z=0.8 from Z=1.5
    cam_xf.AddRotateXYZOp().Set(Gf.Vec3f(75, 0, -135))

    print(f"Camera repositioned at {cam_pos}")
    print(f"Rotation: (75, 0, -135)")

# Set as active camera
import omni.kit.viewport.utility as vp_util
viewport = vp_util.get_active_viewport()
if viewport:
    viewport.set_active_camera(camera_path)
    print(f"Active camera: {camera_path}")

# Where is Cassie?
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    xf = UsdGeom.Xformable(cassie)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            print(f"Cassie at: {op.Get()}")
            break
'''

print("Fixing camera...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(1)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
