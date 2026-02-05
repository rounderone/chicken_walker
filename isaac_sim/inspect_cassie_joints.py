# Inspect ALL of Cassie's joints in detail
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
from pxr import UsdPhysics, UsdGeom

stage = omni.usd.get_context().get_stage()

print("=== CASSIE JOINT INSPECTION ===\\n")

# Find ALL joints
print("All joints in Cassie:")
print("-" * 50)

joints = []
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/Cassie" in path:
        type_name = prim.GetTypeName()
        if "Joint" in type_name:
            joints.append((path, type_name, prim))

for path, type_name, prim in sorted(joints):
    name = path.split("/")[-1]

    # Get connected bodies
    body0 = "?"
    body1 = "?"
    if prim.IsA(UsdPhysics.Joint):
        joint = UsdPhysics.Joint(prim)
        targets0 = joint.GetBody0Rel().GetTargets()
        targets1 = joint.GetBody1Rel().GetTargets()
        if targets0:
            body0 = str(targets0[0]).split("/")[-1]
        if targets1:
            body1 = str(targets1[0]).split("/")[-1]

    print(f"{name}")
    print(f"  Type: {type_name}")
    print(f"  Connects: {body0} -> {body1}")
    print()

# Also check the articulation structure using omni.isaac.core
print("\\n=== Using Articulation API ===\\n")
try:
    from omni.isaac.core.articulations import Articulation

    cassie = Articulation(prim_path="/World/Cassie")
    cassie.initialize()

    print(f"DOF Names: {cassie.dof_names}")
    print(f"Num DOFs: {cassie.num_dof}")

    if cassie.dof_names:
        print("\\nJoint index mapping:")
        for i, name in enumerate(cassie.dof_names):
            print(f"  [{i}] {name}")
except Exception as e:
    print(f"Articulation error: {e}")

# Check for thigh_joint which Isaac Lab uses
print("\\n=== Checking for 'thigh' joints ===")
for path, type_name, prim in joints:
    if "thigh" in path.lower():
        print(f"Found: {path}")
'''

print("Inspecting Cassie joints...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")
