# Simple AT-ST - single rigid body, no joints
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
from pxr import UsdGeom, UsdPhysics, Gf

stage = omni.usd.get_context().get_stage()

# Delete old AT-ST parts and joints
to_delete = []
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "ATST" in path or path.startswith("/World/j_"):
        to_delete.append(path)

for path in to_delete:
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)

print("Cleaned up old parts")

# Create AT-ST as a SINGLE rigid body with child geometry
atst_path = "/World/ATST"
atst = UsdGeom.Xform.Define(stage, atst_path)
atst_prim = stage.GetPrimAtPath(atst_path)

# Position the whole AT-ST
xf = UsdGeom.Xformable(atst_prim)
xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.5))  # Raise it up

# Add physics to the ROOT only
UsdPhysics.RigidBodyAPI.Apply(atst_prim)
UsdPhysics.MassAPI.Apply(atst_prim).CreateMassAttr(10.0)

# Create child geometry (these inherit physics from parent)
def add_part(name, pos, size, color):
    path = atst_path + "/" + name
    cube = UsdGeom.Cube.Define(stage, path)
    prim = stage.GetPrimAtPath(path)

    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([color])

    # Collision on each part
    UsdPhysics.CollisionAPI.Apply(prim)

# Body at center
add_part("body", (0, 0, 0.3), (0.15, 0.12, 0.10), (0.8, 0.2, 0.2))

# Left leg (relative to AT-ST root)
add_part("l_upper", (-0.15, 0, 0.05), (0.03, 0.03, 0.12), (0.2, 0.7, 0.2))
add_part("l_lower", (-0.15, 0, -0.20), (0.025, 0.025, 0.12), (0.2, 0.2, 0.7))
add_part("l_foot", (-0.15, 0, -0.42), (0.08, 0.06, 0.02), (0.7, 0.7, 0.2))

# Right leg
add_part("r_upper", (0.15, 0, 0.05), (0.03, 0.03, 0.12), (0.2, 0.7, 0.2))
add_part("r_lower", (0.15, 0, -0.20), (0.025, 0.025, 0.12), (0.2, 0.2, 0.7))
add_part("r_foot", (0.15, 0, -0.42), (0.08, 0.06, 0.02), (0.7, 0.7, 0.2))

print("Created simple AT-ST (single rigid body)")
print("- Red body")
print("- Green upper legs")
print("- Blue lower legs")
print("- Yellow feet")
print("")
print("Root at Z=0.5, feet at Z=0.08 (above ground)")
print("Press PLAY - should drop and land as one piece!")
'''

print("Creating simple AT-ST...")
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
