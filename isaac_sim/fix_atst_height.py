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

# Script to fix the AT-ST height - move it up so feet are above ground
script = '''
import omni.usd
from pxr import UsdGeom, Gf

stage = omni.usd.get_context().get_stage()

# Get PhysicsATST root and move it higher
# Current: Z=0.8, but feet are at -0.85 relative = -0.05 absolute (below ground!)
# Need: Root Z should be at least 1.0 so feet are at 1.0 - 0.85 = 0.15 (above ground)

atst = stage.GetPrimAtPath("/World/PhysicsATST")
if atst:
    xform = UsdGeom.Xformable(atst)
    xform.ClearXformOpOrder()
    # Move to X=-2.0 (away from Cassie), Z=1.1 (feet will be at 0.25 above ground)
    xform.AddTranslateOp().Set(Gf.Vec3d(-2.0, 0.0, 1.1))
    print("Moved PhysicsATST to (-2.0, 0.0, 1.1)")
    print("Feet should now be at Z=0.25 (above ground)")
else:
    print("PhysicsATST not found!")

# Verify by checking bounding box
from pxr import Usd
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
if atst:
    bound = cache.ComputeWorldBound(atst)
    if not bound.GetRange().IsEmpty():
        mn = bound.GetRange().GetMin()
        mx = bound.GetRange().GetMax()
        print(f"New bbox: min Z={mn[2]:.3f}, max Z={mx[2]:.3f}")
'''

print("Fixing AT-ST height so feet are above ground...")
result = send_command("execute_script", {"code": script})

if result.get("status") == "success":
    print("Success!")
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

print("\nNow try pressing PLAY in Isaac Sim!")
print("The AT-ST should stay on the ground instead of disappearing.")
