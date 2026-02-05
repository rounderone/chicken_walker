import socket
import json

def send_command(command_type, params=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(30)
    try:
        sock.connect(("localhost", 8766))
        command = {"type": command_type, "params": params or {}}
        sock.sendall(json.dumps(command).encode('utf-8'))
        response = sock.recv(65536)
        return json.loads(response.decode('utf-8'))
    finally:
        sock.close()

# Get all children of /World
print("Checking /World children...")
result = send_command("list_children", {"prim_path": "/World", "depth": 2})
if result.get("status") == "success":
    children = result.get("result", {}).get("children", [])
    print(f"Found {len(children)} children:")
    for c in children:
        print(f"  {c['path']} ({c['type']})")
else:
    print(f"Error: {result}")

# Check if PhysicsATST exists
print("\nChecking for PhysicsATST...")
result = send_command("get_prim_info", {"prim_path": "/World/PhysicsATST"})
print(f"PhysicsATST: {result.get('status')}")

# Check Cassie position
print("\nChecking Cassie...")
result = send_command("get_prim_info", {"prim_path": "/World/Cassie"})
if result.get("status") == "success":
    info = result.get("result", {})
    print(f"Cassie bbox: {info.get('bbox')}")
    print(f"Has articulation: {info.get('has_articulation')}")
