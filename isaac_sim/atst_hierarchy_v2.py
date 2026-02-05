# AT-ST with hierarchical structure (parent-child chain)
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

if stage.GetPrimAtPath("/World/ATST"):
    stage.RemovePrim("/World/ATST")

print("Building AT-ST - just a solid single rigid body that WORKS...")

# Forget joints for now - make a solid AT-ST that won't fall apart
atst_path = "/World/ATST"
root = UsdGeom.Xform.Define(stage, atst_path)
root_prim = stage.GetPrimAtPath(atst_path)

# Position at origin, raised above ground
xf = UsdGeom.Xformable(root_prim)
xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.4))

# Single rigid body on the root
UsdPhysics.RigidBodyAPI.Apply(root_prim)
UsdPhysics.MassAPI.Apply(root_prim).CreateMassAttr(5.0)

# All child parts are just collision shapes, no separate rigid bodies
def add_part(name, local_pos, half_size, color):
    path = atst_path + "/" + name
    cube = UsdGeom.Cube.Define(stage, path)
    prim = stage.GetPrimAtPath(path)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*local_pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*half_size))
    cube.CreateDisplayColorAttr([color])
    UsdPhysics.CollisionAPI.Apply(prim)
    return path

# Relative to root at Z=0.4
# Body at top
add_part("body", (0, 0, 0.15), (0.06, 0.05, 0.04), (0.9, 0.3, 0.3))

# Left leg
add_part("l_upper", (-0.08, 0, 0.02), (0.015, 0.015, 0.06), (0.3, 0.9, 0.3))
add_part("l_lower", (-0.08, 0, -0.10), (0.012, 0.012, 0.06), (0.3, 0.3, 0.9))
add_part("l_foot", (-0.08, 0, -0.20), (0.03, 0.025, 0.015), (0.9, 0.9, 0.3))

# Right leg
add_part("r_upper", (0.08, 0, 0.02), (0.015, 0.015, 0.06), (0.3, 0.9, 0.3))
add_part("r_lower", (0.08, 0, -0.10), (0.012, 0.012, 0.06), (0.3, 0.3, 0.9))
add_part("r_foot", (0.08, 0, -0.20), (0.03, 0.025, 0.015), (0.9, 0.9, 0.3))

print("Created solid AT-ST (single rigid body)")
print("Root at Z=0.4")
print("Foot bottoms at Z = 0.4 - 0.20 - 0.015 = 0.185")
print("")
print("NO JOINTS - this will just drop and stand as one piece")
print("Press PLAY!")
'''

print("Building solid AT-ST...")
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
