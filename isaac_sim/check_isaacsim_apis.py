# Check available isaacsim APIs for joint control
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
print("Checking available isaacsim APIs...")

# Check isaacsim.core
try:
    import isaacsim.core
    print(f"isaacsim.core: {dir(isaacsim.core)}")
except Exception as e:
    print(f"isaacsim.core error: {e}")

# Check for Articulation
try:
    from isaacsim.core.articulations import Articulation
    print("\\nArticulation class available!")
    print(f"Articulation methods: {[m for m in dir(Articulation) if not m.startswith('_')][:20]}")
except Exception as e:
    print(f"Articulation error: {e}")

# Check for ArticulationView
try:
    from isaacsim.core.articulations import ArticulationView
    print("\\nArticulationView available!")
except Exception as e:
    print(f"ArticulationView error: {e}")

# Check for Robot
try:
    from isaacsim.core.robots import Robot
    print("\\nRobot class available!")
except Exception as e:
    print(f"Robot error: {e}")

# Check omni.isaac.core
try:
    import omni.isaac.core
    from omni.isaac.core.articulations import Articulation as OmniArticulation
    print("\\nomni.isaac.core.articulations.Articulation available!")
except Exception as e:
    print(f"omni.isaac.core error: {e}")

# Check for DC (dynamics control)
try:
    from omni.isaac.dynamic_control import _dynamic_control
    dc = _dynamic_control.acquire_dynamic_control_interface()
    print("\\nDynamic Control interface available!")
except Exception as e:
    print(f"Dynamic Control error: {e}")

# Check PhysX
try:
    import omni.physx
    print("\\nomni.physx available")
except Exception as e:
    print(f"omni.physx error: {e}")
'''

result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")
