# Build AT-ST with working collision
import socket
import json

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
if stage.GetPrimAtPath("/World/TestCube"):
    stage.RemovePrim("/World/TestCube")

# Remove old AT-ST
if stage.GetPrimAtPath("/World/ATST"):
    stage.RemovePrim("/World/ATST")

print("Building AT-ST...")

# Create AT-ST root
atst_path = "/World/ATST"
root = UsdGeom.Xform.Define(stage, atst_path)

# Positions (ground is at Z=0)
BODY_Z = 1.0      # Body center height
UPPER_Z = 0.65    # Upper leg center
LOWER_Z = 0.30    # Lower leg center
FOOT_Z = 0.08     # Foot center (just above ground)
LEG_X = 0.25      # Leg spread from center

# Colors
GRAY = [(0.5, 0.5, 0.55)]
DARK = [(0.3, 0.3, 0.35)]

def make_part(name, pos, size, color, mass, is_body=False):
    path = atst_path + "/" + name
    cube = UsdGeom.Cube.Define(stage, path)
    cube.AddTranslateOp().Set(Gf.Vec3d(*pos))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr(color)

    prim = stage.GetPrimAtPath(path)

    if is_body:
        # Body is the articulation root
        UsdPhysics.ArticulationRootAPI.Apply(prim)

    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)

    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(mass)

    return path

# === CREATE BODY PARTS ===
body = make_part("body", (0, 0, BODY_Z), (0.2, 0.15, 0.12), GRAY, 5.0, is_body=True)

# Left leg
l_upper = make_part("l_upper", (-LEG_X, 0, UPPER_Z), (0.04, 0.04, 0.15), GRAY, 2.0)
l_lower = make_part("l_lower", (-LEG_X, 0, LOWER_Z), (0.03, 0.03, 0.15), GRAY, 2.0)
l_foot = make_part("l_foot", (-LEG_X, 0, FOOT_Z), (0.12, 0.10, 0.02), DARK, 20.0)

# Right leg
r_upper = make_part("r_upper", (LEG_X, 0, UPPER_Z), (0.04, 0.04, 0.15), GRAY, 2.0)
r_lower = make_part("r_lower", (LEG_X, 0, LOWER_Z), (0.03, 0.03, 0.15), GRAY, 2.0)
r_foot = make_part("r_foot", (LEG_X, 0, FOOT_Z), (0.12, 0.10, 0.02), DARK, 20.0)

print("Created 7 body parts")

# === CREATE JOINTS ===
STIFF = 100000.0
DAMP = 10000.0

def make_joint(name, parent, child, parent_pos, child_pos):
    path = atst_path + "/" + name
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateAxisAttr("X")
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*parent_pos))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*child_pos))

    # Very stiff drive
    prim = stage.GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(STIFF)
    drive.CreateDampingAttr(DAMP)
    drive.CreateTargetPositionAttr(0.0)

    return path

# Joint positions (hip at Z=0.82, knee at Z=0.47, ankle at Z=0.15)
HIP_Z = 0.82
KNEE_Z = 0.47
ANKLE_Z = 0.15

# Left leg joints
make_joint("j_l_hip", body, l_upper,
    (-LEG_X, 0, HIP_Z - BODY_Z),    # relative to body
    (0, 0, HIP_Z - UPPER_Z))         # relative to upper leg

make_joint("j_l_knee", l_upper, l_lower,
    (0, 0, KNEE_Z - UPPER_Z),
    (0, 0, KNEE_Z - LOWER_Z))

make_joint("j_l_ankle", l_lower, l_foot,
    (0, 0, ANKLE_Z - LOWER_Z),
    (0, 0, ANKLE_Z - FOOT_Z))

# Right leg joints
make_joint("j_r_hip", body, r_upper,
    (LEG_X, 0, HIP_Z - BODY_Z),
    (0, 0, HIP_Z - UPPER_Z))

make_joint("j_r_knee", r_upper, r_lower,
    (0, 0, KNEE_Z - UPPER_Z),
    (0, 0, KNEE_Z - LOWER_Z))

make_joint("j_r_ankle", r_lower, r_foot,
    (0, 0, ANKLE_Z - LOWER_Z),
    (0, 0, ANKLE_Z - FOOT_Z))

print("Created 6 joints")
print("")
print("=== AT-ST Ready ===")
print("Feet at Z=0.08 (above ground at Z=0)")
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
