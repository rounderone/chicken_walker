# Take a screenshot from Isaac Sim
import socket
import json
import os

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

output_path = "D:/Projects/ATST/isaac_sim/screenshot.png"

print("Taking screenshot...")
result = send_command("screenshot", {"path": output_path})

if result.get("status") == "success":
    print(f"Screenshot saved to: {output_path}")
    if os.path.exists(output_path):
        print(f"File size: {os.path.getsize(output_path)} bytes")
else:
    print(f"Error: {result.get('message')}")
