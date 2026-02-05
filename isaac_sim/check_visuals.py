# Check visuals folder in Cassie
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
from pxr import UsdGeom, Gf

stage = omni.usd.get_context().get_stage()

# Check visuals folder
visuals = stage.GetPrimAtPath("/World/Cassie/pelvis/visuals")
if visuals:
    print("Pelvis visuals found:")
    print(f"  Type: {visuals.GetTypeName()}")
    print(f"  Children: {len(list(visuals.GetChildren()))}")

    for child in visuals.GetChildren():
        print(f"    {child.GetPath()} - {child.GetTypeName()}")

        # Check if this child has meshes
        if child.IsA(UsdGeom.Mesh):
            print("      IS A MESH!")
        elif child.IsA(UsdGeom.Gprim):
            print("      Is a Gprim")

        # Check its children
        for subchild in child.GetChildren():
            print(f"      {subchild.GetPath()} - {subchild.GetTypeName()}")
            if subchild.IsA(UsdGeom.Mesh):
                print("        IS A MESH!")

# Let's also traverse ALL prims and find any mesh-like things
print("\\nAll geometry prims under Cassie:")
for prim in stage.Traverse():
    path_str = str(prim.GetPath())
    if "/World/Cassie" in path_str:
        type_name = prim.GetTypeName()
        if type_name in ["Mesh", "Cube", "Sphere", "Cylinder", "Capsule", "Cone"]:
            print(f"  {path_str} - {type_name}")
        elif prim.IsA(UsdGeom.Gprim):
            print(f"  {path_str} - {type_name} (Gprim)")

# Check if there's a UsdGeom purpose issue (render vs proxy vs guide)
print("\\nChecking purpose attributes...")
visuals = stage.GetPrimAtPath("/World/Cassie/pelvis/visuals")
if visuals:
    imageable = UsdGeom.Imageable(visuals)
    purpose = imageable.GetPurposeAttr().Get()
    print(f"Visuals purpose: {purpose}")

    vis = imageable.GetVisibilityAttr().Get()
    print(f"Visuals visibility: {vis}")
'''

print("Checking Cassie visuals...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")
