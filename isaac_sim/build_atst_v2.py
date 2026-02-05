# Build AT-ST with correct transforms
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

# Remove test cube
if stage.GetPrimAtPath("/World/Cube"):
    stage.RemovePrim("/World/Cube")

print("Building AT-ST...")

# Heights (ground at Z=0)
FOOT_Z = 0.05
LOWER_Z = 0.25
UPPER_Z = 0.55
BODY_Z = 0.80
LEG_X = 0.20

def create_part(path, pos, size, color, mass, is_root=False):
    """Create a cube part with physics"""
    cube = UsdGeom.Cube.Define(stage, path)
    prim = stage.GetPrimAtPath(path)

    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([color])

    if is_root:
        UsdPhysics.ArticulationRootAPI.Apply(prim)

    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)

    return path

# Create parts
body = create_part("/World/ATST_body", (0, 0, BODY_Z), (0.15, 0.12, 0.10), (0.8, 0.2, 0.2), 3.0, is_root=True)

l_upper = create_part("/World/ATST_l_upper", (-LEG_X, 0, UPPER_Z), (0.03, 0.03, 0.12), (0.2, 0.7, 0.2), 1.5)
l_lower = create_part("/World/ATST_l_lower", (-LEG_X, 0, LOWER_Z), (0.025, 0.025, 0.12), (0.2, 0.2, 0.7), 1.5)
l_foot = create_part("/World/ATST_l_foot", (-LEG_X, 0, FOOT_Z), (0.08, 0.06, 0.02), (0.7, 0.7, 0.2), 15.0)

r_upper = create_part("/World/ATST_r_upper", (LEG_X, 0, UPPER_Z), (0.03, 0.03, 0.12), (0.2, 0.7, 0.2), 1.5)
r_lower = create_part("/World/ATST_r_lower", (LEG_X, 0, LOWER_Z), (0.025, 0.025, 0.12), (0.2, 0.2, 0.7), 1.5)
r_foot = create_part("/World/ATST_r_foot", (LEG_X, 0, FOOT_Z), (0.08, 0.06, 0.02), (0.7, 0.7, 0.2), 15.0)

print("Created 7 parts")

# Joint settings
STIFF = 50000.0
DAMP = 5000.0

def create_joint(path, parent, child, p_anchor, c_anchor):
    """Create revolute joint with drive"""
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateAxisAttr("X")
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*p_anchor))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*c_anchor))

    prim = stage.GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(STIFF)
    drive.CreateDampingAttr(DAMP)
    drive.CreateTargetPositionAttr(0.0)

# Joint heights
HIP_Z = 0.68
KNEE_Z = 0.40
ANKLE_Z = 0.12

# Left leg joints
create_joint("/World/j_l_hip", body, l_upper,
    (-LEG_X, 0, HIP_Z - BODY_Z),
    (0, 0, HIP_Z - UPPER_Z))

create_joint("/World/j_l_knee", l_upper, l_lower,
    (0, 0, KNEE_Z - UPPER_Z),
    (0, 0, KNEE_Z - LOWER_Z))

create_joint("/World/j_l_ankle", l_lower, l_foot,
    (0, 0, ANKLE_Z - LOWER_Z),
    (0, 0, ANKLE_Z - FOOT_Z))

# Right leg joints
create_joint("/World/j_r_hip", body, r_upper,
    (LEG_X, 0, HIP_Z - BODY_Z),
    (0, 0, HIP_Z - UPPER_Z))

create_joint("/World/j_r_knee", r_upper, r_lower,
    (0, 0, KNEE_Z - UPPER_Z),
    (0, 0, KNEE_Z - LOWER_Z))

create_joint("/World/j_r_ankle", r_lower, r_foot,
    (0, 0, ANKLE_Z - LOWER_Z),
    (0, 0, ANKLE_Z - FOOT_Z))

print("Created 6 joints")

# Verify positions
for name in ["body", "l_foot", "r_foot"]:
    prim = stage.GetPrimAtPath(f"/World/ATST_{name}")
    if prim:
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if "translate" in op.GetOpName():
                print(f"  {name}: {op.Get()}")

print("")
print("=== AT-ST Ready ===")
print("Red body, green upper legs, blue lower legs, yellow feet")
print("Press PLAY to test!")
'''

print("Building AT-ST...")
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
