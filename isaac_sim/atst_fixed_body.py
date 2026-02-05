# AT-ST with fixed body - only legs can move
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

print("Building AT-ST with FIXED body...")

atst_path = "/World/ATST"
UsdGeom.Xform.Define(stage, atst_path)

# The key insight: child body origin should BE at joint location
# So upper leg origin is at hip, lower leg origin is at knee, foot origin is at ankle

BODY_Z = 0.55
HIP_Z = 0.51      # Where hips connect (body bottom)
KNEE_Z = 0.37     # Where knees connect
ANKLE_Z = 0.23    # Where ankles connect
FOOT_BOTTOM = 0.08

LEG_X = 0.10
STIFF = 10000.0
DAMP = 500.0

def make_body(name, pos, half_size, color, mass, kinematic=False):
    path = atst_path + "/" + name
    cube = UsdGeom.Cube.Define(stage, path)
    prim = stage.GetPrimAtPath(path)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*half_size))
    cube.CreateDisplayColorAttr([color])

    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    if kinematic:
        rb.CreateKinematicEnabledAttr(True)

    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    return path

# Body - KINEMATIC (won't fall, legs hang from it)
body = make_body("body", (0, 0, BODY_Z), (0.07, 0.05, 0.04), (0.9, 0.3, 0.3), 2.0, kinematic=True)
UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(body))

# Upper legs - origin at HIP position, visual extends down
# Upper leg half-height = 0.07, so center offset = -0.07
l_up = make_body("l_upper", (-LEG_X, 0, HIP_Z - 0.07), (0.018, 0.018, 0.07), (0.3, 0.9, 0.3), 0.5)
r_up = make_body("r_upper", (LEG_X, 0, HIP_Z - 0.07), (0.018, 0.018, 0.07), (0.3, 0.9, 0.3), 0.5)

# Lower legs - similar pattern
l_lo = make_body("l_lower", (-LEG_X, 0, KNEE_Z - 0.07), (0.015, 0.015, 0.07), (0.3, 0.3, 0.9), 0.5)
r_lo = make_body("r_lower", (LEG_X, 0, KNEE_Z - 0.07), (0.015, 0.015, 0.07), (0.3, 0.3, 0.9), 0.5)

# Feet
l_ft = make_body("l_foot", (-LEG_X, 0, ANKLE_Z - 0.07), (0.035, 0.028, 0.02), (0.9, 0.9, 0.3), 2.0)
r_ft = make_body("r_foot", (LEG_X, 0, ANKLE_Z - 0.07), (0.035, 0.028, 0.02), (0.9, 0.9, 0.3), 2.0)

print("  Body is KINEMATIC (fixed in space)")
print("  6 leg parts can move")

# Joints
joints_path = atst_path + "/joints"
UsdGeom.Scope.Define(stage, joints_path)

def make_joint(name, parent, child, anchor):
    path = joints_path + "/" + name
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateAxisAttr("X")
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*anchor))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0.07))  # Top of child leg

    prim = stage.GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(STIFF)
    drive.CreateDampingAttr(DAMP)
    drive.CreateTargetPositionAttr(0.0)

# Hip joints - anchor at body bottom
make_joint("hip_l", body, l_up, (-LEG_X, 0, HIP_Z - BODY_Z))
make_joint("hip_r", body, r_up, (LEG_X, 0, HIP_Z - BODY_Z))

# Knee joints - anchor at upper leg bottom
make_joint("knee_l", l_up, l_lo, (0, 0, -0.07))
make_joint("knee_r", r_up, r_lo, (0, 0, -0.07))

# Ankle joints
make_joint("ankle_l", l_lo, l_ft, (0, 0, -0.07))
make_joint("ankle_r", r_lo, r_ft, (0, 0, -0.07))

print("  6 joints")
print("")
print("Press PLAY - body stays fixed, legs should hang down!")
'''

print("Building AT-ST with fixed body...")
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
