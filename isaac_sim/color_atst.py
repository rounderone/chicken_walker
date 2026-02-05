# Make AT-ST more visible
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

# Color the AT-ST parts bright colors
colors = {
    "/World/ATST/body": (0.8, 0.2, 0.2),      # Red body
    "/World/ATST/l_upper": (0.2, 0.8, 0.2),   # Green upper legs
    "/World/ATST/r_upper": (0.2, 0.8, 0.2),
    "/World/ATST/l_lower": (0.2, 0.2, 0.8),   # Blue lower legs
    "/World/ATST/r_lower": (0.2, 0.2, 0.8),
    "/World/ATST/l_foot": (0.8, 0.8, 0.2),    # Yellow feet
    "/World/ATST/r_foot": (0.8, 0.8, 0.2),
}

for path, color in colors.items():
    prim = stage.GetPrimAtPath(path)
    if prim:
        cube = UsdGeom.Cube(prim)
        cube.CreateDisplayColorAttr([color])
        print(f"Colored {path.split('/')[-1]}")
    else:
        print(f"NOT FOUND: {path}")

# Make ground darker
ground = stage.GetPrimAtPath("/World/MyGround")
if ground:
    UsdGeom.Cube(ground).CreateDisplayColorAttr([(0.15, 0.15, 0.18)])
    print("Darkened ground")

print("\\nAT-ST now has bright colors - should be easy to see")
'''

result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)

time.sleep(0.5)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
