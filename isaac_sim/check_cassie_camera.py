# Check Cassie status and reposition camera
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

stage = omni.usd.get_context().get_stage()

# Check if Cassie exists
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    print("Cassie exists in scene")
    # Get its position
    xf = UsdGeom.Xformable(cassie)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            print(f"Cassie position: {op.Get()}")
            break
else:
    print("Cassie NOT found - will reload it")

# List all prims at /World level
print("\\nPrims at /World:")
world = stage.GetPrimAtPath("/World")
if world:
    for child in world.GetChildren():
        print(f"  {child.GetPath()}")

# Set viewport camera to look at origin from good angle
import omni.kit.viewport.utility as vp_util
viewport = vp_util.get_active_viewport()
if viewport:
    from omni.kit.viewport.utility import frame_viewport_selection
    # Set camera to look at Cassie location
    viewport.set_camera_position_target(
        position=(4, -4, 3),  # Camera position
        target=(0, 0, 1)      # Look at point (where Cassie is)
    )
    print("\\nCamera repositioned to view Cassie area")
'''

print("Checking Cassie and camera...")
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
