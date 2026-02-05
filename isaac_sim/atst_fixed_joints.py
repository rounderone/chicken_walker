# AT-ST with FIXED joints (no rotation) to test stability
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

# Clean up all ATST related prims
to_delete = []
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "ATST" in path or "joint" in path.lower():
        to_delete.append(path)
for path in sorted(to_delete, reverse=True):  # Delete children first
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)

print("Building AT-ST with fixed joints...")

# Parts should TOUCH at joint locations
# Body: center at Z=0.70, half-height=0.08 → bottom at 0.62
# Upper leg: top at 0.62, half-height=0.10 → center at 0.52
# Lower leg: top at 0.42, half-height=0.10 → center at 0.32
# Foot: top at 0.22, half-height=0.015 → center at 0.205

BODY_Z = 0.70
UPPER_Z = 0.52
LOWER_Z = 0.32
FOOT_Z = 0.05  # Keep feet above ground
LEG_X = 0.12

def make_part(path, pos, half_size, color, mass):
    cube = UsdGeom.Cube.Define(stage, path)
    prim = stage.GetPrimAtPath(path)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*half_size))
    cube.CreateDisplayColorAttr([color])
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    return path

# Body (articulation root)
body = make_part("/World/ATST_body", (0, 0, BODY_Z), (0.10, 0.08, 0.08), (0.9, 0.3, 0.3), 2.0)
UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(body))

# Legs
l_up = make_part("/World/ATST_l_up", (-LEG_X, 0, UPPER_Z), (0.02, 0.02, 0.10), (0.3, 0.9, 0.3), 1.0)
l_lo = make_part("/World/ATST_l_lo", (-LEG_X, 0, LOWER_Z), (0.02, 0.02, 0.10), (0.3, 0.3, 0.9), 1.0)
l_ft = make_part("/World/ATST_l_ft", (-LEG_X, 0, FOOT_Z), (0.05, 0.04, 0.02), (0.9, 0.9, 0.3), 8.0)

r_up = make_part("/World/ATST_r_up", (LEG_X, 0, UPPER_Z), (0.02, 0.02, 0.10), (0.3, 0.9, 0.3), 1.0)
r_lo = make_part("/World/ATST_r_lo", (LEG_X, 0, LOWER_Z), (0.02, 0.02, 0.10), (0.3, 0.3, 0.9), 1.0)
r_ft = make_part("/World/ATST_r_ft", (LEG_X, 0, FOOT_Z), (0.05, 0.04, 0.02), (0.9, 0.9, 0.3), 8.0)

print("Created 7 parts")

# Use FIXED joints - parts won't move relative to each other
def make_fixed_joint(name, body0, body1, pos0, pos1):
    path = "/World/fj_" + name
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*pos0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*pos1))
    return path

# Joint at body-upper connection (Z=0.62)
HIP_Z = 0.62
make_fixed_joint("l_hip", body, l_up, (-LEG_X, 0, HIP_Z-BODY_Z), (0, 0, HIP_Z-UPPER_Z))
make_fixed_joint("r_hip", body, r_up, (LEG_X, 0, HIP_Z-BODY_Z), (0, 0, HIP_Z-UPPER_Z))

# Joint at upper-lower connection (Z=0.42)
KNEE_Z = 0.42
make_fixed_joint("l_knee", l_up, l_lo, (0, 0, KNEE_Z-UPPER_Z), (0, 0, KNEE_Z-LOWER_Z))
make_fixed_joint("r_knee", r_up, r_lo, (0, 0, KNEE_Z-UPPER_Z), (0, 0, KNEE_Z-LOWER_Z))

# Joint at lower-foot connection (Z=0.22)
ANKLE_Z = 0.22
make_fixed_joint("l_ankle", l_lo, l_ft, (0, 0, ANKLE_Z-LOWER_Z), (0, 0, ANKLE_Z-FOOT_Z))
make_fixed_joint("r_ankle", r_lo, r_ft, (0, 0, ANKLE_Z-LOWER_Z), (0, 0, ANKLE_Z-FOOT_Z))

print("Created 6 FIXED joints")
print("")
print("This AT-ST has rigid joints (no bending)")
print("If it stays together, we can add movable joints next")
print("Press PLAY to test!")
'''

print("Building AT-ST with fixed joints...")
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
