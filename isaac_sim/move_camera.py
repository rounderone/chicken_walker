# Move camera to see objects at origin
import socket
import json
import time

def send_command(command_type, params=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(60)
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
from pxr import UsdGeom, Gf, Sdf

stage = omni.usd.get_context().get_stage()

# Find camera
camera_path = "/OmniverseKit_Persp"
camera = stage.GetPrimAtPath(camera_path)

if camera:
    xf = UsdGeom.Xformable(camera)

    # Get existing ops and modify them
    ops = xf.GetOrderedXformOps()
    print(f"Camera has {len(ops)} xform ops:")
    for op in ops:
        print(f"  {op.GetOpName()}: {op.Get()}")

    # Try setting via the ops directly
    for op in ops:
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(3.0, -3.0, 2.0))
            print("Set camera translate to (3, -3, 2)")
        elif "rotate" in op.GetOpName():
            op.Set(Gf.Vec3f(60, 0, 45))
            print("Set camera rotate")
else:
    print("Camera not found")

# Check what's in the scene
print("\\nObjects in /World:")
world = stage.GetPrimAtPath("/World")
if world:
    for child in world.GetChildren():
        path = str(child.GetPath())
        if "Ground" not in path and "Physics_Mat" not in path:
            print(f"  {path}")
'''

print("Moving camera...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

# Wait for viewport to update
time.sleep(2)

# Take screenshot
print("\nTaking screenshot...")
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print(f"Screenshot saved")
