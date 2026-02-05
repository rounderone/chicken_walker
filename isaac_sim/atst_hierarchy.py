# AT-ST with proper USD hierarchy (like Cassie robot)
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

# Clean up everything
to_delete = []
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/" in path and "Ground" not in path and "defaultGround" not in path:
        to_delete.append(path)
for path in sorted(set(to_delete), reverse=True):
    if stage.GetPrimAtPath(path):
        try:
            stage.RemovePrim(path)
        except:
            pass

print("Building AT-ST with proper hierarchy...")

# Create robot root
robot_path = "/World/ATST"
robot = UsdGeom.Xform.Define(stage, robot_path)
robot_prim = stage.GetPrimAtPath(robot_path)

# Articulation root on the robot
UsdPhysics.ArticulationRootAPI.Apply(robot_prim)

# Position robot above ground
xf = UsdGeom.Xformable(robot_prim)
xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))

# Body - child of robot, this is the "base link"
body_path = robot_path + "/body"
body = UsdGeom.Cube.Define(stage, body_path)
body_prim = stage.GetPrimAtPath(body_path)
body_xf = UsdGeom.Xformable(body_prim)
body_xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.6))
body_xf.AddScaleOp().Set(Gf.Vec3f(0.10, 0.08, 0.06))
body.CreateDisplayColorAttr([(0.9, 0.3, 0.3)])
UsdPhysics.RigidBodyAPI.Apply(body_prim)
UsdPhysics.CollisionAPI.Apply(body_prim)
UsdPhysics.MassAPI.Apply(body_prim).CreateMassAttr(2.0)

# Left leg parts - as siblings under robot root
LEG_X = 0.10

# Left upper leg
l_up_path = robot_path + "/l_upper"
l_up = UsdGeom.Cube.Define(stage, l_up_path)
l_up_prim = stage.GetPrimAtPath(l_up_path)
l_up_xf = UsdGeom.Xformable(l_up_prim)
l_up_xf.AddTranslateOp().Set(Gf.Vec3d(-LEG_X, 0, 0.42))
l_up_xf.AddScaleOp().Set(Gf.Vec3f(0.02, 0.02, 0.08))
l_up.CreateDisplayColorAttr([(0.3, 0.9, 0.3)])
UsdPhysics.RigidBodyAPI.Apply(l_up_prim)
UsdPhysics.CollisionAPI.Apply(l_up_prim)
UsdPhysics.MassAPI.Apply(l_up_prim).CreateMassAttr(0.5)

# Left lower leg
l_lo_path = robot_path + "/l_lower"
l_lo = UsdGeom.Cube.Define(stage, l_lo_path)
l_lo_prim = stage.GetPrimAtPath(l_lo_path)
l_lo_xf = UsdGeom.Xformable(l_lo_prim)
l_lo_xf.AddTranslateOp().Set(Gf.Vec3d(-LEG_X, 0, 0.26))
l_lo_xf.AddScaleOp().Set(Gf.Vec3f(0.015, 0.015, 0.08))
l_lo.CreateDisplayColorAttr([(0.3, 0.3, 0.9)])
UsdPhysics.RigidBodyAPI.Apply(l_lo_prim)
UsdPhysics.CollisionAPI.Apply(l_lo_prim)
UsdPhysics.MassAPI.Apply(l_lo_prim).CreateMassAttr(0.5)

# Left foot
l_ft_path = robot_path + "/l_foot"
l_ft = UsdGeom.Cube.Define(stage, l_ft_path)
l_ft_prim = stage.GetPrimAtPath(l_ft_path)
l_ft_xf = UsdGeom.Xformable(l_ft_prim)
l_ft_xf.AddTranslateOp().Set(Gf.Vec3d(-LEG_X, 0, 0.10))
l_ft_xf.AddScaleOp().Set(Gf.Vec3f(0.04, 0.03, 0.02))
l_ft.CreateDisplayColorAttr([(0.9, 0.9, 0.3)])
UsdPhysics.RigidBodyAPI.Apply(l_ft_prim)
UsdPhysics.CollisionAPI.Apply(l_ft_prim)
UsdPhysics.MassAPI.Apply(l_ft_prim).CreateMassAttr(5.0)

# Right upper leg
r_up_path = robot_path + "/r_upper"
r_up = UsdGeom.Cube.Define(stage, r_up_path)
r_up_prim = stage.GetPrimAtPath(r_up_path)
r_up_xf = UsdGeom.Xformable(r_up_prim)
r_up_xf.AddTranslateOp().Set(Gf.Vec3d(LEG_X, 0, 0.42))
r_up_xf.AddScaleOp().Set(Gf.Vec3f(0.02, 0.02, 0.08))
r_up.CreateDisplayColorAttr([(0.3, 0.9, 0.3)])
UsdPhysics.RigidBodyAPI.Apply(r_up_prim)
UsdPhysics.CollisionAPI.Apply(r_up_prim)
UsdPhysics.MassAPI.Apply(r_up_prim).CreateMassAttr(0.5)

# Right lower leg
r_lo_path = robot_path + "/r_lower"
r_lo = UsdGeom.Cube.Define(stage, r_lo_path)
r_lo_prim = stage.GetPrimAtPath(r_lo_path)
r_lo_xf = UsdGeom.Xformable(r_lo_prim)
r_lo_xf.AddTranslateOp().Set(Gf.Vec3d(LEG_X, 0, 0.26))
r_lo_xf.AddScaleOp().Set(Gf.Vec3f(0.015, 0.015, 0.08))
r_lo.CreateDisplayColorAttr([(0.3, 0.3, 0.9)])
UsdPhysics.RigidBodyAPI.Apply(r_lo_prim)
UsdPhysics.CollisionAPI.Apply(r_lo_prim)
UsdPhysics.MassAPI.Apply(r_lo_prim).CreateMassAttr(0.5)

# Right foot
r_ft_path = robot_path + "/r_foot"
r_ft = UsdGeom.Cube.Define(stage, r_ft_path)
r_ft_prim = stage.GetPrimAtPath(r_ft_path)
r_ft_xf = UsdGeom.Xformable(r_ft_prim)
r_ft_xf.AddTranslateOp().Set(Gf.Vec3d(LEG_X, 0, 0.10))
r_ft_xf.AddScaleOp().Set(Gf.Vec3f(0.04, 0.03, 0.02))
r_ft.CreateDisplayColorAttr([(0.9, 0.9, 0.3)])
UsdPhysics.RigidBodyAPI.Apply(r_ft_prim)
UsdPhysics.CollisionAPI.Apply(r_ft_prim)
UsdPhysics.MassAPI.Apply(r_ft_prim).CreateMassAttr(5.0)

print("Created 7 body parts under /World/ATST/")

# Create joints scope
joints_path = robot_path + "/joints"
UsdGeom.Scope.Define(stage, joints_path)

# Joint helper
def make_joint(name, parent_path, child_path, anchor_in_parent, anchor_in_child):
    jpath = joints_path + "/" + name
    joint = UsdPhysics.RevoluteJoint.Define(stage, jpath)
    joint.CreateBody0Rel().SetTargets([parent_path])
    joint.CreateBody1Rel().SetTargets([child_path])
    joint.CreateAxisAttr("X")
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*anchor_in_parent))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*anchor_in_child))

    # Add drive
    jprim = stage.GetPrimAtPath(jpath)
    drive = UsdPhysics.DriveAPI.Apply(jprim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(50000.0)
    drive.CreateDampingAttr(5000.0)
    drive.CreateTargetPositionAttr(0.0)

    # Exclude collision between connected bodies
    joint.CreateExcludeFromArticulationAttr(False)

    return jpath

# Calculate joint positions
# Body at Z=0.6, upper at Z=0.42, lower at Z=0.26, foot at Z=0.10
# Hip joint where body meets upper: Z = 0.54 (body bottom)
# Knee joint where upper meets lower: Z = 0.34 (upper bottom)
# Ankle joint where lower meets foot: Z = 0.18 (lower bottom)

# Left joints
make_joint("l_hip", body_path, l_up_path,
    (-LEG_X, 0, 0.54 - 0.6),   # anchor in body local coords
    (0, 0, 0.54 - 0.42))       # anchor in upper local coords

make_joint("l_knee", l_up_path, l_lo_path,
    (0, 0, 0.34 - 0.42),
    (0, 0, 0.34 - 0.26))

make_joint("l_ankle", l_lo_path, l_ft_path,
    (0, 0, 0.18 - 0.26),
    (0, 0, 0.18 - 0.10))

# Right joints
make_joint("r_hip", body_path, r_up_path,
    (LEG_X, 0, 0.54 - 0.6),
    (0, 0, 0.54 - 0.42))

make_joint("r_knee", r_up_path, r_lo_path,
    (0, 0, 0.34 - 0.42),
    (0, 0, 0.34 - 0.26))

make_joint("r_ankle", r_lo_path, r_ft_path,
    (0, 0, 0.18 - 0.26),
    (0, 0, 0.18 - 0.10))

print("Created 6 revolute joints under /World/ATST/joints/")
print("")
print("Feet at Z=0.10, foot bottom at ~0.08 (above ground)")
print("Press PLAY to test!")
'''

print("Building AT-ST with hierarchy...")
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
