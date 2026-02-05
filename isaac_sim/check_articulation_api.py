# Check Articulation API methods
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
from omni.isaac.core.articulations import Articulation

# Get all public methods
methods = [m for m in dir(Articulation) if not m.startswith('_')]
print("Articulation methods:")
for m in methods:
    print(f"  {m}")
'''

result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    print(result.get("result", {}).get("stdout", ""))
