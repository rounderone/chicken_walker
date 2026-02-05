# Position Cassie to stand and stiffen joints
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

cassie = stage.GetPrimAtPath("/World/Cassie")
if cassie:
    # Raise Cassie - her feet are at about Z=-1.1, so raising by 1.2 puts feet at Z=0.1
    xf = UsdGeom.Xformable(cassie)
    for op in xf.GetOrderedXformOps():
        if "translate" in op.GetOpName():
            op.Set(Gf.Vec3d(0, 0, 1.3))
            print("Cassie raised to Z=1.3 (feet should be just above ground)")
            break

    # Increase joint stiffness to help her stand
    STIFFNESS_MULT = 50.0
    DAMPING_MULT = 100.0

    joints_modified = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "/World/Cassie" in path and "Joint" in prim.GetTypeName():
            if prim.HasAPI(UsdPhysics.DriveAPI):
                drive = UsdPhysics.DriveAPI.Get(prim, "angular")
                if drive:
                    stiff_attr = drive.GetStiffnessAttr()
                    damp_attr = drive.GetDampingAttr()

                    if stiff_attr and stiff_attr.Get():
                        old_stiff = stiff_attr.Get()
                        new_stiff = old_stiff * STIFFNESS_MULT
                        stiff_attr.Set(new_stiff)

                    if damp_attr and damp_attr.Get():
                        old_damp = damp_attr.Get()
                        new_damp = old_damp * DAMPING_MULT
                        damp_attr.Set(new_damp)

                    joints_modified += 1

    print(f"Modified {joints_modified} joint drives (stiffness x{STIFFNESS_MULT}, damping x{DAMPING_MULT})")

    # Count meshes
    mesh_count = 0
    for prim in stage.Traverse():
        if "/World/Cassie" in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh):
            mesh_count += 1
    print(f"Cassie has {mesh_count} meshes")

    print("")
    print("Press PLAY - Cassie should try to hold her pose!")
else:
    print("Cassie not found")
'''

print("Setting up Cassie to stand...")
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
