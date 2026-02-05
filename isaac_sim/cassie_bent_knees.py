# Cassie with bent knees for shock absorption
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
import math

stage = omni.usd.get_context().get_stage()

cassie = stage.GetPrimAtPath("/World/Cassie")
if not cassie:
    print("Cassie not found!")
else:
    print("Setting up Cassie with bent knees...")

    # Position - start a bit higher to give room to drop
    xf = UsdGeom.Xformable(cassie)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 1.0))
            print("Position: Z=1.0")
            break

    # Joint settings - moderate stiffness with good damping for shock absorption
    # Lower stiffness = more compliant/springy joints
    STIFFNESS = 500.0   # Not too stiff - allow flex
    DAMPING = 100.0     # Good damping to absorb energy

    # Target angles for bent-knee stance (in degrees, converted to radians for USD)
    # Cassie's joints need specific angles for her reverse-knee (digitigrade) legs
    joint_targets = {
        # Hip flexion - slight forward lean
        "hip_abduction": 0.0,
        "hip_rotation": 0.0,
        "hip_flexion": math.radians(10),  # Slight forward

        # Knee - bent backward (this is the main shock absorber)
        "knee": math.radians(-30),  # Bent knee

        # Ankle/shin - compensate to keep foot flat
        "shin": math.radians(20),

        # Tarsus (lower leg segment)
        "tarsus": math.radians(-10),

        # Toe
        "toe": math.radians(0),
    }

    joints_configured = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "/World/Cassie" not in path:
            continue

        if "Joint" not in prim.GetTypeName():
            continue

        joint_name = prim.GetName().lower()

        # Apply drive settings
        if prim.HasAPI(UsdPhysics.DriveAPI):
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if not drive:
                # Try to apply it
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")

            if drive:
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(STIFFNESS)
                drive.CreateDampingAttr(DAMPING)

                # Set target angle based on joint name
                target_angle = 0.0
                for key, angle in joint_targets.items():
                    if key in joint_name:
                        target_angle = angle
                        break

                # USD uses degrees for target position
                drive.CreateTargetPositionAttr(math.degrees(target_angle))

                joints_configured += 1
                if target_angle != 0:
                    print(f"  {joint_name}: target={math.degrees(target_angle):.1f} deg")

    print(f"\\nConfigured {joints_configured} joints")
    print(f"Stiffness: {STIFFNESS}, Damping: {DAMPING}")
    print("")
    print("Press PLAY - Cassie should drop and absorb with bent knees!")
'''

print("Setting up Cassie with bent knees...")
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
