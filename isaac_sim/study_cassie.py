# Study Cassie's joint structure
import socket
import json

def send_command(command_type, params=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(60)
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

print("=== Studying Cassie's Structure ===")
print("")

# Find articulation root
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/Cassie" in path and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(f"ArticulationRoot: {path}")
        break

# Find all joints
print("")
print("Joints:")
joints = []
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/Cassie" in path:
        typename = prim.GetTypeName()
        if "Joint" in typename:
            joints.append((path, typename))

for path, typename in joints[:8]:  # First 8 joints
    print(f"  {path.split('/')[-1]}: {typename}")

    prim = stage.GetPrimAtPath(path)
    joint = UsdPhysics.RevoluteJoint(prim) if "Revolute" in typename else None

    if joint:
        # Get body relationships
        body0 = joint.GetBody0Rel().GetTargets()
        body1 = joint.GetBody1Rel().GetTargets()
        axis = joint.GetAxisAttr().Get() if joint.GetAxisAttr() else "?"
        pos0 = joint.GetLocalPos0Attr().Get() if joint.GetLocalPos0Attr() else None
        pos1 = joint.GetLocalPos1Attr().Get() if joint.GetLocalPos1Attr() else None

        print(f"    Body0: {body0}")
        print(f"    Body1: {body1}")
        print(f"    Axis: {axis}")
        print(f"    LocalPos0: {pos0}")
        print(f"    LocalPos1: {pos1}")

        # Check for drive
        if prim.HasAPI(UsdPhysics.DriveAPI):
            # Get the drive
            drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drive_api:
                stiff = drive_api.GetStiffnessAttr().Get() if drive_api.GetStiffnessAttr() else 0
                damp = drive_api.GetDampingAttr().Get() if drive_api.GetDampingAttr() else 0
                print(f"    Drive: stiffness={stiff}, damping={damp}")
        print("")

print(f"Total joints: {len(joints)}")
'''

print("Studying Cassie structure...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")
