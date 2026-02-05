# Zoom out to see AT-ST
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
from pxr import UsdGeom, Gf

stage = omni.usd.get_context().get_stage()

# Set camera to see AT-ST from the side
camera_path = "/OmniverseKit_Persp"
camera = stage.GetPrimAtPath(camera_path)

if camera:
    xf = UsdGeom.Xformable(camera)
    ops = xf.GetOrderedXformOps()

    for op in ops:
        if "translate" in op.GetOpName():
            # Position: in front and to the side, looking at origin
            op.Set(Gf.Vec3d(3.0, -3.0, 1.5))
            print("Camera moved to (3, -3, 1.5)")
        elif "rotate" in op.GetOpName():
            # Look towards origin
            op.Set(Gf.Vec3f(70, 0, 135))
            print("Camera rotated")

# Frame the AT-ST
try:
    import omni.kit.commands
    omni.kit.commands.execute("FrameAllCommand")
    print("Framed all")
except:
    pass

print("Camera adjusted - AT-ST should be visible")
'''

result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)

time.sleep(1)

# Take screenshot
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("Screenshot taken")
