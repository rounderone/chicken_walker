# Set camera to see the scene and take screenshot
import socket
import json

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
from pxr import UsdGeom, Gf, Usd

stage = omni.usd.get_context().get_stage()

# List all prims to see what exists
print("Prims in scene:")
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if path.startswith("/World") and not "Ground" in path and not "Physics_Mat" in path:
        print(f"  {path}")

# Set camera position to see the pendulum
camera_path = "/OmniverseKit_Persp"
camera_prim = stage.GetPrimAtPath(camera_path)
if camera_prim:
    xf = UsdGeom.Xformable(camera_prim)
    xf.ClearXformOpOrder()
    # Position camera to look at origin from front-right, elevated
    xf.AddTranslateOp().Set(Gf.Vec3d(5.0, -5.0, 3.0))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(60, 0, 45))
    print("Camera repositioned")
else:
    print(f"Camera not found at {camera_path}")
'''

print("Setting camera and checking scene...")
result = send_command("execute_script", {"code": script})

if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

# Take screenshot
print("\nTaking screenshot...")
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print(f"Screenshot: {result.get('status')}")
