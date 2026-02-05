# Build AT-ST using Cassie's joint pattern
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

# Remove Cassie and old AT-ST
for path in ["/World/Cassie", "/World/ATST"]:
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)

print("Building AT-ST like Cassie...")

# Create AT-ST root (just an Xform container, NOT articulation root)
atst_path = "/World/ATST"
atst = UsdGeom.Xform.Define(stage, atst_path)

# Body dimensions
BODY_Z = 0.6     # Body center height
LEG_X = 0.10    # Leg X offset from center

# Cassie-style stiffness/damping
STIFF = 300.0
DAMP = 0.1

def make_body(name, pos, half_size, color, mass, is_artic_root=False):
    path = atst_path + "/" + name
    cube = UsdGeom.Cube.Define(stage, path)
    prim = stage.GetPrimAtPath(path)

    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*half_size))
    cube.CreateDisplayColorAttr([color])

    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)

    if is_artic_root:
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        print(f"  ArticulationRoot on {name}")

    return path

# Create body - THIS gets the ArticulationRoot (like Cassie's pelvis)
body = make_body("body", (0, 0, BODY_Z), (0.08, 0.06, 0.05), (0.9, 0.3, 0.3), 2.0, is_artic_root=True)

# Left leg bodies - positioned relative to world
# Upper leg: joint at body bottom, extends down
l_upper = make_body("l_upper", (-LEG_X, 0, 0.45), (0.02, 0.02, 0.08), (0.3, 0.9, 0.3), 0.5)
l_lower = make_body("l_lower", (-LEG_X, 0, 0.28), (0.015, 0.015, 0.08), (0.3, 0.3, 0.9), 0.5)
l_foot = make_body("l_foot", (-LEG_X, 0, 0.12), (0.04, 0.03, 0.02), (0.9, 0.9, 0.3), 3.0)

# Right leg
r_upper = make_body("r_upper", (LEG_X, 0, 0.45), (0.02, 0.02, 0.08), (0.3, 0.9, 0.3), 0.5)
r_lower = make_body("r_lower", (LEG_X, 0, 0.28), (0.015, 0.015, 0.08), (0.3, 0.3, 0.9), 0.5)
r_foot = make_body("r_foot", (LEG_X, 0, 0.12), (0.04, 0.03, 0.02), (0.9, 0.9, 0.3), 3.0)

print("  Created 7 bodies")

# Joints scope
joints_path = atst_path + "/joints"
UsdGeom.Scope.Define(stage, joints_path)

def make_joint(name, parent_path, child_path, parent_anchor):
    """Create joint with LocalPos1=(0,0,0) like Cassie"""
    path = joints_path + "/" + name
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([parent_path])
    joint.CreateBody1Rel().SetTargets([child_path])
    joint.CreateAxisAttr("X")
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*parent_anchor))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))  # Always zero!

    # Cassie-style drive
    prim = stage.GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(STIFF)
    drive.CreateDampingAttr(DAMP)
    drive.CreateTargetPositionAttr(0.0)

    return path

# Body bottom at Z = 0.6 - 0.05 = 0.55
# Upper leg center at Z = 0.45, so joint in body at (-0.10, 0, 0.55-0.6) = (-0.10, 0, -0.05)
make_joint("hip_left", body, l_upper, (-LEG_X, 0, -0.05))
make_joint("hip_right", body, r_upper, (LEG_X, 0, -0.05))

# Upper leg bottom at Z = 0.45 - 0.08 = 0.37
# Lower leg center at 0.28, so joint in upper at (0, 0, 0.37-0.45) = (0, 0, -0.08)
make_joint("knee_left", l_upper, l_lower, (0, 0, -0.08))
make_joint("knee_right", r_upper, r_lower, (0, 0, -0.08))

# Lower leg bottom at Z = 0.28 - 0.08 = 0.20
# Foot center at 0.12, so joint in lower at (0, 0, 0.20-0.28) = (0, 0, -0.08)
make_joint("ankle_left", l_lower, l_foot, (0, 0, -0.08))
make_joint("ankle_right", r_lower, r_foot, (0, 0, -0.08))

print("  Created 6 joints (Cassie-style)")
print("")
print("Foot bottoms at Z = 0.12 - 0.02 = 0.10 (above ground)")
print("Press PLAY!")
'''

print("Building AT-ST like Cassie...")
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
