# Study Cassie's joint structure
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
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()

print("Cassie's joints:\\n")

for prim in stage.Traverse():
    path = str(prim.GetPath())
    if "/World/Cassie" not in path:
        continue

    type_name = prim.GetTypeName()
    if "Joint" in type_name:
        name = prim.GetName()

        # Get connected bodies
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            body0 = joint.GetBody0Rel().GetTargets()
            body1 = joint.GetBody1Rel().GetTargets()

            b0_name = str(body0[0]).split("/")[-1] if body0 else "none"
            b1_name = str(body1[0]).split("/")[-1] if body1 else "none"

            # Get drive settings
            stiff = "?"
            damp = "?"
            target = "?"
            if prim.HasAPI(UsdPhysics.DriveAPI):
                drive = UsdPhysics.DriveAPI.Get(prim, "angular")
                if drive:
                    s = drive.GetStiffnessAttr().Get()
                    d = drive.GetDampingAttr().Get()
                    t = drive.GetTargetPositionAttr().Get()
                    stiff = f"{s:.0f}" if s else "0"
                    damp = f"{d:.0f}" if d else "0"
                    target = f"{t:.1f}" if t is not None else "0"

            print(f"{name}")
            print(f"  {b0_name} -> {b1_name}")
            print(f"  stiff={stiff}, damp={damp}, target={target}")
            print()
'''

result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")
