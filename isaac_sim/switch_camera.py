# Switch viewport to ViewCamera and take screenshot
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
import omni.kit.viewport.utility as vp_util

viewport = vp_util.get_active_viewport()
if viewport:
    # Set the active camera to our ViewCamera
    viewport.set_active_camera("/World/ViewCamera")
    print("Switched to /World/ViewCamera")
else:
    print("No active viewport")

# Also print what's in the scene
import omni.usd
stage = omni.usd.get_context().get_stage()

world = stage.GetPrimAtPath("/World")
if world:
    print("\\nScene contents:")
    for child in world.GetChildren():
        print(f"  {child.GetPath()}")
'''

print("Switching camera...")
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
