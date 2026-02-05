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

# Get current positions
print("Getting bounding boxes...")

result = send_command("get_prim_info", {"prim_path": "/World/PhysicsATST"})
if result.get("status") == "success":
    info = result.get("result", {})
    print(f"PhysicsATST bbox: {info.get('bbox')}")

result = send_command("get_prim_info", {"prim_path": "/World/Cassie"})
if result.get("status") == "success":
    info = result.get("result", {})
    print(f"Cassie bbox: {info.get('bbox')}")

# Move PhysicsATST to a new position away from Cassie
print("\nMoving PhysicsATST to X=-2.0 to avoid Cassie...")

# Use execute_script to properly reposition
script = '''
import omni.usd
from pxr import UsdGeom, Gf

stage = omni.usd.get_context().get_stage()

# Get PhysicsATST root
atst = stage.GetPrimAtPath("/World/PhysicsATST")
if atst:
    xform = UsdGeom.Xformable(atst)
    # Clear existing transform ops
    xform.ClearXformOpOrder()
    # Set new position: X=-2.0, Y=0, Z=0.8 (above ground)
    xform.AddTranslateOp().Set(Gf.Vec3d(-2.0, 0.0, 0.8))
    print("Moved PhysicsATST to (-2.0, 0.0, 0.8)")
else:
    print("PhysicsATST not found!")

# Also check Cassie position
cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    xform = UsdGeom.Xformable(cassie)
    ops = xform.GetOrderedXformOps()
    if ops:
        for op in ops:
            print(f"Cassie xform op: {op.GetOpName()} = {op.Get()}")
    else:
        print("Cassie has no xform ops")
'''

result = send_command("execute_script", {"code": script})
print(f"Result: {result.get('status')}")
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(f"Output: {stdout}")
else:
    print(f"Error: {result.get('message')}")
    if result.get('traceback'):
        print(result.get('traceback'))

# Verify new positions
print("\nVerifying new positions...")
result = send_command("get_prim_info", {"prim_path": "/World/PhysicsATST"})
if result.get("status") == "success":
    info = result.get("result", {})
    print(f"PhysicsATST bbox: {info.get('bbox')}")
