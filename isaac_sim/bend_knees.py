# Set bent knee targets for Cassie
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

print("Setting bent knee targets...")

# Target angles in DEGREES for each joint type
# Cassie's legs: thigh -> knee -> shin -> tarsus -> toe
# The "reverse knee" bend is primarily at ankle_joint (shin to tarsus)
joint_targets = {
    "hip_flexion": 5,       # Slight forward lean at hip
    "knee_joint": -15,      # Upper knee bends back slightly
    "knee_to_shin": -10,    # Part of knee mechanism
    "ankle_joint": 40,      # MAIN reverse-knee bend (shin to tarsus)
    "toe_joint": -20,       # Toe angle to keep foot flat
}

# Stiffness settings - softer for shock absorption
joint_stiffness = {
    "hip": 600,
    "knee": 400,      # Softer knees = more shock absorption
    "ankle": 500,     # Medium for the main bend
    "toe": 300,
}

joints_set = 0
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/Cassie" not in path:
        continue

    if "Joint" not in prim.GetTypeName():
        continue

    name = prim.GetName().lower()

    if prim.HasAPI(UsdPhysics.DriveAPI):
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if drive:
            # Find matching target angle
            target = 0.0
            for key, angle in joint_targets.items():
                if key in name:
                    target = angle
                    break

            # Find matching stiffness
            stiff = 500  # default
            for key, s in joint_stiffness.items():
                if key in name:
                    stiff = s
                    break

            drive.CreateStiffnessAttr(stiff)
            drive.CreateDampingAttr(stiff * 0.2)  # 20% of stiffness
            drive.CreateTargetPositionAttr(target)

            if target != 0:
                print(f"  {name}: target={target} deg, stiff={stiff}")

            joints_set += 1

print(f"\\nConfigured {joints_set} joints with bent-knee stance")
print("")
print("Press PLAY - knees should bend like shock absorbers!")
'''

print("Setting bent knee targets...")
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
