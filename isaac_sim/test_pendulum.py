# Test basic joint behavior with a simple pendulum
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

# Clean up old stuff
for path in ["/World/PhysicsATST", "/World/Cassie", "/World/MiniATST", "/World/Pendulum"]:
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
        print(f"Removed {path}")

# Create a simple pendulum to test joint behavior
pend_path = "/World/Pendulum"
root = UsdGeom.Xform.Define(stage, pend_path)

# Top anchor - fixed in space (kinematic rigid body)
top_path = pend_path + "/top"
top = UsdGeom.Cube.Define(stage, top_path)
top.AddTranslateOp().Set(Gf.Vec3d(0, 0, 2.0))  # 2m above ground
top.AddScaleOp().Set(Gf.Vec3f(0.1, 0.1, 0.1))  # Small cube

top_prim = stage.GetPrimAtPath(top_path)
rb = UsdPhysics.RigidBodyAPI.Apply(top_prim)
rb.CreateKinematicEnabledAttr(True)  # Fixed in place
UsdPhysics.CollisionAPI.Apply(top_prim)

# Swinging arm
arm_path = pend_path + "/arm"
arm = UsdGeom.Cube.Define(stage, arm_path)
arm.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.5))  # Below top
arm.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.4))  # Long thin arm

arm_prim = stage.GetPrimAtPath(arm_path)
UsdPhysics.RigidBodyAPI.Apply(arm_prim)
UsdPhysics.CollisionAPI.Apply(arm_prim)
mass = UsdPhysics.MassAPI.Apply(arm_prim)
mass.CreateMassAttr(1.0)

# Joint connecting top to arm
joint_path = pend_path + "/joint"
joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
joint.CreateBody0Rel().SetTargets([top_path])
joint.CreateBody1Rel().SetTargets([arm_path])
joint.CreateAxisAttr("Y")  # Swing in X-Z plane

# Anchor positions:
# Top cube is at Z=2.0, arm center is at Z=1.5
# Joint should be at the connection point
# Top anchor: bottom of top cube (Z = 2.0 - 0.1 = 1.9 in world, or -0.1 in local)
# Arm anchor: top of arm (Z = 1.5 + 0.4 = 1.9 in world, or +0.4 in local)
joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, -0.1))
joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0.4))

print("Created pendulum test")
print("- Top cube (kinematic) at Z=2.0")
print("- Arm (dynamic) at Z=1.5")
print("- Joint connecting them")
print("")
print("Press PLAY - arm should swing like a pendulum!")
'''

print("Creating pendulum test...")
result = send_command("execute_script", {"code": script})

if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")
