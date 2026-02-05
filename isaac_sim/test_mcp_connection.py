"""
Test MCP connection to Isaac Sim
Run this AFTER enabling the MCP extension in Isaac Sim
"""
import socket
import json

def send_command(command_type, params=None):
    """Send a command to Isaac Sim MCP server"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect(("localhost", 8766))
        command = {"type": command_type, "params": params or {}}
        sock.sendall(json.dumps(command).encode('utf-8'))
        sock.settimeout(30.0)
        response = sock.recv(16384)
        return json.loads(response.decode('utf-8'))
    finally:
        sock.close()

if __name__ == "__main__":
    print("Testing MCP connection to Isaac Sim...")
    try:
        result = send_command("get_scene_info")
        print(f"SUCCESS! Connected to Isaac Sim")
        print(f"Response: {json.dumps(result, indent=2)}")
    except ConnectionRefusedError:
        print("ERROR: Could not connect to Isaac Sim MCP server")
        print("Make sure:")
        print("  1. Isaac Sim is running")
        print("  2. The MCP extension is enabled (Window > Extensions)")
    except Exception as e:
        print(f"ERROR: {e}")
