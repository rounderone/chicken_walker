# Show Cassie collision geometry as visible
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
from pxr import UsdGeom, Gf, UsdPhysics

stage = omni.usd.get_context().get_stage()

# Find all collision prims and make them visible + colorful
print("Making collision geometry visible...")

colors = [
    (0.9, 0.3, 0.3),  # Red
    (0.3, 0.9, 0.3),  # Green
    (0.3, 0.3, 0.9),  # Blue
    (0.9, 0.9, 0.3),  # Yellow
    (0.9, 0.3, 0.9),  # Magenta
    (0.3, 0.9, 0.9),  # Cyan
]

color_idx = 0
geom_count = 0

for prim in stage.Traverse():
    path_str = str(prim.GetPath())
    if "/World/Cassie" in path_str:
        # Check if it's geometry
        if prim.IsA(UsdGeom.Gprim):
            geom = UsdGeom.Gprim(prim)

            # Set visibility
            imageable = UsdGeom.Imageable(prim)
            imageable.GetVisibilityAttr().Set("inherited")

            # Set purpose to default (renderable)
            imageable.GetPurposeAttr().Set(UsdGeom.Tokens.default_)

            # Add color
            if hasattr(geom, 'CreateDisplayColorAttr'):
                geom.CreateDisplayColorAttr([colors[color_idx % len(colors)]])

            print(f"  {path_str} - {prim.GetTypeName()}")
            geom_count += 1
            color_idx += 1

print(f"\\nMade {geom_count} collision shapes visible")

# Also check if Cassie's root has visibility issues
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    imageable = UsdGeom.Imageable(cassie)
    imageable.GetVisibilityAttr().Set("inherited")

# Position Cassie
xf = UsdGeom.Xformable(cassie)
for op in xf.GetOrderedXformOps():
    if "translate" in op.GetOpName():
        op.Set(Gf.Vec3d(0, 0, 1.2))
        break

print("\\nCassie positioned at Z=1.2")
print("Press PLAY to see physics simulation!")
'''

print("Making Cassie visible...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
