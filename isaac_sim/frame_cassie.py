# Frame Cassie in viewport
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
import omni.kit.viewport.utility as vp_util
from pxr import Sdf

stage = omni.usd.get_context().get_stage()

# Switch back to perspective camera
viewport = vp_util.get_active_viewport()
if viewport:
    # Use default perspective camera
    viewport.set_active_camera("/OmniverseKit_Persp")
    print("Switched to perspective camera")

    # Try to frame on Cassie using selection
    from omni.kit.viewport.utility import frame_viewport_selection

    # Select Cassie
    ctx = omni.usd.get_context()
    ctx.get_selection().set_selected_prim_paths(["/World/Cassie"], True)
    print("Selected /World/Cassie")

    # Frame the selection
    try:
        frame_viewport_selection(viewport)
        print("Framed viewport on selection")
    except Exception as e:
        print(f"Frame error: {e}")

# Verify Cassie exists and has geometry
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    children = list(cassie.GetChildren())
    print(f"Cassie has {len(children)} direct children")
    if children:
        print(f"First few: {[str(c.GetPath()) for c in children[:3]]}")
'''

print("Framing Cassie...")
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
