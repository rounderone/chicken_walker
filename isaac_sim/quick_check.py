import socket
import json
import sys

OUTPUT_FILE = "D:/Projects/ATST/isaac_sim/output.txt"

def log(msg):
    print(msg)
    with open(OUTPUT_FILE, "a") as f:
        f.write(msg + "\n")

# Clear output file
with open(OUTPUT_FILE, "w") as f:
    f.write("")

def send_command(command_type, params=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    try:
        sock.connect(("localhost", 8766))
        command = {"type": command_type, "params": params or {}}
        sock.sendall(json.dumps(command).encode('utf-8'))
        response = sock.recv(65536)
        return json.loads(response.decode('utf-8'))
    finally:
        sock.close()

try:
    result = send_command("get_scene_info")
    log("Connected to Isaac Sim!")
    log(f"Status: {result.get('status')}")
    prims = result.get('result', {}).get('prims', [])
    log(f"Prim count: {len(prims)}")
    for p in prims:
        log(f"  {p}")
except ConnectionRefusedError:
    log("Isaac Sim MCP not running - please start Isaac Sim first")
    sys.exit(1)
except socket.timeout:
    log("Connection timed out - Isaac Sim may not be responding")
    sys.exit(1)
except Exception as e:
    log(f"Error: {e}")
    sys.exit(1)
