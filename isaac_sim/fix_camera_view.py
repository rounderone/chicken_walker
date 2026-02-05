# Fix camera to see the AT-ST
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
from pxr import UsdGeom, Gf, Usd

stage = omni.usd.get_context().get_stage()

# First, let's check what exists and where
print("Checking AT-ST parts:")
atst_parts = []
for name in ["body", "l_upper", "r_upper", "l_lower", "r_lower", "l_foot", "r_foot"]:
    path = f"/World/ATST/{name}"
    prim = stage.GetPrimAtPath(path)
    if prim:
        xf = UsdGeom.Xformable(prim)
        ops = xf.GetOrderedXformOps()
        for op in ops:
            if "translate" in op.GetOpName():
                pos = op.Get()
                print(f"  {name}: {pos}")
                atst_parts.append(name)
                break

print(f"Found {len(atst_parts)} parts")

# Try using the viewport API properly
try:
    import omni.kit.viewport.utility as vp_util
    from omni.kit.viewport.utility import get_active_viewport_and_window

    viewport_api, window = get_active_viewport_and_window()
    if viewport_api:
        # Set camera position and target
        from pxr import Gf
        camera_pos = Gf.Vec3d(4, -4, 3)
        target_pos = Gf.Vec3d(0, 0, 0.5)

        # Use viewport API to set view
        viewport_api.set_camera_position("/OmniverseKit_Persp", camera_pos, True)
        viewport_api.set_camera_target("/OmniverseKit_Persp", target_pos, True)
        print(f"Set camera via viewport API")
except Exception as e:
    print(f"Viewport API method failed: {e}")

# Fallback: try setting camera transform directly with correct values
try:
    camera = stage.GetPrimAtPath("/OmniverseKit_Persp")
    if camera:
        # Clear and reset transforms
        xf = UsdGeom.Xformable(camera)
        xf.ClearXformOpOrder()

        # Set position looking at origin from front-right
        xf.AddTranslateOp().Set(Gf.Vec3d(4.0, -4.0, 2.5))

        # Rotation to look at origin (pitch down, yaw to face origin)
        xf.AddRotateXYZOp().Set(Gf.Vec3f(65, 0, 135))

        print("Reset camera transform directly")
except Exception as e:
    print(f"Direct transform failed: {e}")

# Force viewport refresh
try:
    import omni.kit.app
    omni.kit.app.get_app().update()
    print("Forced app update")
except:
    pass

print("\\nCamera should now see the AT-ST at origin")
'''

print("Fixing camera view...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

# Wait longer for viewport to update
print("\nWaiting for viewport update...")
time.sleep(2)

# Take multiple screenshots to ensure we get the updated frame
for i in range(3):
    time.sleep(0.5)
    result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})

print("Screenshot taken")
