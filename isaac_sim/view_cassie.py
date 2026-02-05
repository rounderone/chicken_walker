# Directly position perspective camera to view Cassie
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
import omni.kit.commands
from pxr import Gf

# Use viewport camera manipulation
import omni.kit.viewport.utility as vp_util

viewport = vp_util.get_active_viewport()
if viewport:
    # Get the camera path
    cam_path = viewport.get_active_camera()
    print(f"Active camera: {cam_path}")

    # Try using the viewport's camera state API
    from omni.kit.viewport.utility.camera_state import ViewportCameraState

    camera_state = ViewportCameraState(cam_path, viewport)

    # Set position and target
    # Camera at (3, -3, 2) looking at (0, 0, 1)
    camera_state.set_position_world(Gf.Vec3d(3.0, -3.0, 2.0), True)
    camera_state.set_target_world(Gf.Vec3d(0, 0, 1.0), True)

    print("Camera set to view Cassie at origin")
    print(f"Position: (3, -3, 2), Target: (0, 0, 1)")
'''

print("Setting camera to view Cassie...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(2)  # Give more time for camera to update
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
