# Configure Cassie's joints for standing
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
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()

cassie = stage.GetPrimAtPath("/World/Cassie")
if not cassie:
    print("Cassie not found!")
else:
    print("Configuring joints for shock absorption...")

    # Moderate stiffness - stiff enough to hold pose, soft enough to flex
    # High damping - absorbs energy on landing
    STIFFNESS = 800.0
    DAMPING = 150.0

    joints_configured = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "/World/Cassie" not in path:
            continue

        if "Joint" not in prim.GetTypeName():
            continue

        # Get or create angular drive
        if not prim.HasAPI(UsdPhysics.DriveAPI):
            UsdPhysics.DriveAPI.Apply(prim, "angular")

        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if drive:
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(STIFFNESS)
            drive.CreateDampingAttr(DAMPING)
            drive.CreateTargetPositionAttr(0.0)  # Hold current pose
            joints_configured += 1

    print(f"Configured {joints_configured} joints")
    print(f"  Stiffness: {STIFFNESS} (springy enough to flex)")
    print(f"  Damping: {DAMPING} (absorbs landing impact)")
    print("")
    print("Press PLAY - Cassie should stand and absorb any small movements!")
'''

print("Configuring Cassie's joints...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(1)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken - press PLAY in Isaac Sim!")
