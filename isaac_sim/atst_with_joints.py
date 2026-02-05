# AT-ST with articulated joints
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

# Clean up
if stage.GetPrimAtPath("/World/ATST"):
    stage.RemovePrim("/World/ATST")
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if path.startswith("/World/joint"):
        stage.RemovePrim(path)

print("Building articulated AT-ST...")

# Measurements
BODY_Z = 0.75
UPPER_Z = 0.50
LOWER_Z = 0.25
FOOT_Z = 0.03
LEG_X = 0.15

# Joint positions
HIP_Z = 0.62
KNEE_Z = 0.37
ANKLE_Z = 0.10

# Very stiff joints
STIFF = 100000.0
DAMP = 10000.0

def make_cube(path, pos, size, color, mass):
    cube = UsdGeom.Cube.Define(stage, path)
    prim = stage.GetPrimAtPath(path)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([color])
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    return path

def make_joint(name, body0, body1, pos0, pos1):
    path = "/World/joint_" + name
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateAxisAttr("X")
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*pos0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*pos1))

    # Stiff drive to hold position
    prim = stage.GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(STIFF)
    drive.CreateDampingAttr(DAMP)
    drive.CreateTargetPositionAttr(0.0)
    return path

# Create body (articulation root)
body = make_cube("/World/ATST_body", (0, 0, BODY_Z), (0.12, 0.10, 0.08), (0.9, 0.3, 0.3), 2.0)
UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(body))

# Create legs
l_upper = make_cube("/World/ATST_l_upper", (-LEG_X, 0, UPPER_Z), (0.025, 0.025, 0.10), (0.3, 0.8, 0.3), 1.0)
l_lower = make_cube("/World/ATST_l_lower", (-LEG_X, 0, LOWER_Z), (0.02, 0.02, 0.10), (0.3, 0.3, 0.8), 1.0)
l_foot = make_cube("/World/ATST_l_foot", (-LEG_X, 0, FOOT_Z), (0.06, 0.05, 0.015), (0.9, 0.9, 0.3), 10.0)

r_upper = make_cube("/World/ATST_r_upper", (LEG_X, 0, UPPER_Z), (0.025, 0.025, 0.10), (0.3, 0.8, 0.3), 1.0)
r_lower = make_cube("/World/ATST_r_lower", (LEG_X, 0, LOWER_Z), (0.02, 0.02, 0.10), (0.3, 0.3, 0.8), 1.0)
r_foot = make_cube("/World/ATST_r_foot", (LEG_X, 0, FOOT_Z), (0.06, 0.05, 0.015), (0.9, 0.9, 0.3), 10.0)

print("Created 7 parts")

# Create joints - anchor positions must match!
make_joint("l_hip", body, l_upper,
    (-LEG_X, 0, HIP_Z - BODY_Z),  # On body: (-0.15, 0, -0.13)
    (0, 0, HIP_Z - UPPER_Z))       # On upper: (0, 0, 0.12)

make_joint("l_knee", l_upper, l_lower,
    (0, 0, KNEE_Z - UPPER_Z),      # On upper: (0, 0, -0.13)
    (0, 0, KNEE_Z - LOWER_Z))      # On lower: (0, 0, 0.12)

make_joint("l_ankle", l_lower, l_foot,
    (0, 0, ANKLE_Z - LOWER_Z),     # On lower: (0, 0, -0.15)
    (0, 0, ANKLE_Z - FOOT_Z))      # On foot: (0, 0, 0.07)

make_joint("r_hip", body, r_upper,
    (LEG_X, 0, HIP_Z - BODY_Z),
    (0, 0, HIP_Z - UPPER_Z))

make_joint("r_knee", r_upper, r_lower,
    (0, 0, KNEE_Z - UPPER_Z),
    (0, 0, KNEE_Z - LOWER_Z))

make_joint("r_ankle", r_lower, r_foot,
    (0, 0, ANKLE_Z - LOWER_Z),
    (0, 0, ANKLE_Z - FOOT_Z))

print("Created 6 joints")
print("")
print("Body: 2kg, Feet: 10kg each")
print("Joint stiffness: 100000")
print("Feet at Z=0.03 (above ground)")
print("")
print("Press PLAY to test!")
'''

print("Building articulated AT-ST...")
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
