# Full scene with ground and AT-ST
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
from pxr import UsdGeom, UsdPhysics, UsdLux, Gf

stage = omni.usd.get_context().get_stage()

# Only remove ATST, keep ground
if stage.GetPrimAtPath("/World/ATST"):
    stage.RemovePrim("/World/ATST")

# Ensure ground exists
if not stage.GetPrimAtPath("/World/Ground"):
    print("Creating ground...")
    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    gprim = stage.GetPrimAtPath("/World/Ground")
    gxf = UsdGeom.Xformable(gprim)
    gxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.25))
    gxf.AddScaleOp().Set(Gf.Vec3f(25, 25, 0.25))
    ground.CreateDisplayColorAttr([(0.3, 0.3, 0.35)])
    UsdPhysics.CollisionAPI.Apply(gprim)

# Ensure light exists
if not stage.GetPrimAtPath("/World/Light"):
    print("Creating light...")
    light = UsdLux.DistantLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(3000)

print("Building AT-ST (Cassie-style joints)...")

atst_path = "/World/ATST"
atst = UsdGeom.Xform.Define(stage, atst_path)

BODY_Z = 0.55
LEG_X = 0.10
STIFF = 300.0
DAMP = 0.1

def make_body(name, pos, half_size, color, mass, is_root=False):
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
    if is_root:
        UsdPhysics.ArticulationRootAPI.Apply(prim)
    return path

# Body with ArticulationRoot
body = make_body("body", (0, 0, BODY_Z), (0.07, 0.05, 0.04), (0.9, 0.3, 0.3), 2.0, is_root=True)

# Legs - upper at Z=0.42, lower at Z=0.26, foot at Z=0.10
l_up = make_body("l_upper", (-LEG_X, 0, 0.42), (0.018, 0.018, 0.07), (0.3, 0.9, 0.3), 0.5)
l_lo = make_body("l_lower", (-LEG_X, 0, 0.26), (0.015, 0.015, 0.07), (0.3, 0.3, 0.9), 0.5)
l_ft = make_body("l_foot", (-LEG_X, 0, 0.10), (0.035, 0.028, 0.018), (0.9, 0.9, 0.3), 2.0)

r_up = make_body("r_upper", (LEG_X, 0, 0.42), (0.018, 0.018, 0.07), (0.3, 0.9, 0.3), 0.5)
r_lo = make_body("r_lower", (LEG_X, 0, 0.26), (0.015, 0.015, 0.07), (0.3, 0.3, 0.9), 0.5)
r_ft = make_body("r_foot", (LEG_X, 0, 0.10), (0.035, 0.028, 0.018), (0.9, 0.9, 0.3), 2.0)

print("  7 bodies created")

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
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

    prim = stage.GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(STIFF)
    drive.CreateDampingAttr(DAMP)
    drive.CreateTargetPositionAttr(0.0)

# Body center at 0.55, bottom at 0.55-0.04=0.51
# Upper center at 0.42
# Anchor in body: 0.51 relative to body center 0.55 = -0.04, but at leg X position
make_joint("hip_l", body, l_up, (-LEG_X, 0, -0.04))
make_joint("hip_r", body, r_up, (LEG_X, 0, -0.04))

# Upper center at 0.42, bottom at 0.42-0.07=0.35
# Lower center at 0.26
# Anchor in upper: 0.35 relative to 0.42 = -0.07
make_joint("knee_l", l_up, l_lo, (0, 0, -0.07))
make_joint("knee_r", r_up, r_lo, (0, 0, -0.07))

# Lower center at 0.26, bottom at 0.26-0.07=0.19
# Foot center at 0.10
# Anchor in lower: 0.19 relative to 0.26 = -0.07
make_joint("ankle_l", l_lo, l_ft, (0, 0, -0.07))
make_joint("ankle_r", r_lo, r_ft, (0, 0, -0.07))

print("  6 joints created (Cassie-style)")
print("")
print("Foot bottom at Z = 0.10 - 0.018 = 0.082 (above ground)")
print("Press PLAY to test!")
'''

print("Creating full scene...")
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
