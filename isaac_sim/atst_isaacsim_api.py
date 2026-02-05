# Use Isaac Sim's high-level API to create articulated robot
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
from pxr import UsdGeom, UsdPhysics, Gf, PhysxSchema

stage = omni.usd.get_context().get_stage()

# Clean up
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/ATST" in path:
        stage.RemovePrim("/World/ATST")
        break

print("Trying Isaac Sim articulation API...")

# Try using isaacsim APIs if available
try:
    from isaacsim.core.prims import RigidPrim, GeometryPrim
    from isaacsim.core.articulations import Articulation
    print("IsaacSim core APIs available")
except ImportError as e:
    print(f"IsaacSim APIs not available: {e}")

# Let's just create a SIMPLE working robot first
# Single body with child collision shapes - NO JOINTS
# This definitely works based on earlier test

robot_path = "/World/ATST"
robot = UsdGeom.Xform.Define(stage, robot_path)
robot_prim = stage.GetPrimAtPath(robot_path)

# Make the whole robot a single rigid body
UsdPhysics.RigidBodyAPI.Apply(robot_prim)
UsdPhysics.MassAPI.Apply(robot_prim).CreateMassAttr(10.0)

# Position it
xf = UsdGeom.Xformable(robot_prim)
xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.5))

# Add collision shapes as children (they inherit the rigid body)
def add_shape(name, local_pos, half_size, color):
    path = robot_path + "/" + name
    cube = UsdGeom.Cube.Define(stage, path)
    prim = stage.GetPrimAtPath(path)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*local_pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*half_size))
    cube.CreateDisplayColorAttr([color])
    UsdPhysics.CollisionAPI.Apply(prim)
    return path

# All positions relative to robot root at Z=0.5
# So robot root at 0.5, body visual at 0.5 + 0.15 = 0.65
add_shape("body", (0, 0, 0.15), (0.08, 0.06, 0.05), (0.9, 0.3, 0.3))

# Legs (relative to root at Z=0.5)
add_shape("l_upper", (-0.08, 0, 0), (0.015, 0.015, 0.08), (0.3, 0.9, 0.3))
add_shape("l_lower", (-0.08, 0, -0.16), (0.012, 0.012, 0.08), (0.3, 0.3, 0.9))
add_shape("l_foot", (-0.08, 0, -0.32), (0.04, 0.03, 0.015), (0.9, 0.9, 0.3))

add_shape("r_upper", (0.08, 0, 0), (0.015, 0.015, 0.08), (0.3, 0.9, 0.3))
add_shape("r_lower", (0.08, 0, -0.16), (0.012, 0.012, 0.08), (0.3, 0.3, 0.9))
add_shape("r_foot", (0.08, 0, -0.32), (0.04, 0.03, 0.015), (0.9, 0.9, 0.3))

print("")
print("Created solid AT-ST (single rigid body, no joints)")
print("Robot root at Z=0.5")
print("Feet at Z = 0.5 - 0.32 = 0.18 (above ground)")
print("")
print("This version has NO JOINTS - legs don't bend")
print("But it WILL stay together when you press PLAY")
print("")
print("If this works, we can explore other ways to add movement")
'''

print("Creating solid AT-ST...")
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
