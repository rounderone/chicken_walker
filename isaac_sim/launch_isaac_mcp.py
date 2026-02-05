"""
Launch Isaac Sim with MCP server running directly
Run with: D:\Projects\ATST\venv_isaaclab\Scripts\python.exe D:\Projects\ATST\isaac_sim\launch_isaac_mcp.py
"""
import sys
import os
import socket
import json
import threading
import traceback
import queue

print("=" * 60)
print("Launching Isaac Sim with MCP Server")
print("=" * 60)
sys.stdout.flush()

from isaacsim import SimulationApp

# Launch Isaac Sim
config = {"headless": False}
print("Starting SimulationApp...")
sys.stdout.flush()
app = SimulationApp(config)

# Import after app starts
import omni
import omni.kit.commands
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

# Command queue for thread-safe execution
command_queue = queue.Queue()
response_queues = {}  # Map request_id to response queue

class MCPServer:
    def __init__(self, host="localhost", port=8766):
        self.host = host
        self.port = port
        self.running = False
        self.socket = None
        self.thread = None
        self.request_counter = 0

    def start(self):
        self.running = True
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.listen(1)
        self.socket.settimeout(1.0)

        self.thread = threading.Thread(target=self._server_loop, daemon=True)
        self.thread.start()
        print(f"MCP server started on {self.host}:{self.port}")

    def stop(self):
        self.running = False
        if self.socket:
            self.socket.close()

    def _server_loop(self):
        while self.running:
            try:
                client, addr = self.socket.accept()
                print(f"Client connected: {addr}")
                threading.Thread(target=self._handle_client, args=(client,), daemon=True).start()
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Server error: {e}")

    def _handle_client(self, client):
        client.settimeout(None)
        buffer = b''
        try:
            while self.running:
                data = client.recv(16384)
                if not data:
                    break
                buffer += data
                try:
                    command = json.loads(buffer.decode('utf-8'))
                    buffer = b''

                    # Create unique request ID
                    self.request_counter += 1
                    request_id = self.request_counter

                    # Create response queue for this request
                    resp_queue = queue.Queue()
                    response_queues[request_id] = resp_queue

                    # Queue command for main thread execution
                    command_queue.put((request_id, command))

                    # Wait for response (timeout 60s)
                    try:
                        response = resp_queue.get(timeout=60)
                    except queue.Empty:
                        response = {"status": "error", "message": "Timeout waiting for response"}
                    finally:
                        del response_queues[request_id]

                    client.sendall(json.dumps(response).encode('utf-8'))
                except json.JSONDecodeError:
                    continue
        except Exception as e:
            print(f"Client error: {e}")
        finally:
            client.close()

# Command executors (run on main thread)
def execute_command(command):
    cmd_type = command.get("type")
    params = command.get("params", {})

    try:
        if cmd_type == "get_scene_info":
            return get_scene_info()
        elif cmd_type == "get_prim_info":
            return get_prim_info(params)
        elif cmd_type == "list_children":
            return list_children(params)
        elif cmd_type == "set_transform":
            return set_transform(params)
        elif cmd_type == "delete_prim":
            return delete_prim(params)
        elif cmd_type == "execute_script":
            return execute_script(params.get("code", ""))
        elif cmd_type == "create_physics_scene":
            return create_physics_scene(params)
        elif cmd_type == "create_prim":
            return create_prim(params)
        elif cmd_type == "load_usd":
            return load_usd(params)
        elif cmd_type == "add_physics":
            return add_physics(params)
        elif cmd_type == "screenshot":
            return take_screenshot(params)
        elif cmd_type == "create_articulation":
            return create_articulation(params)
        elif cmd_type == "set_joint_states":
            return set_joint_states(params)
        elif cmd_type == "get_joint_info":
            return get_joint_info(params)
        elif cmd_type == "step_physics":
            return step_physics(params)
        else:
            return {"status": "error", "message": f"Unknown command: {cmd_type}"}
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}

def get_scene_info():
    stage = omni.usd.get_context().get_stage()
    prims = [str(p.GetPath()) for p in stage.Traverse()]
    return {
        "status": "success",
        "result": {
            "prim_count": len(prims),
            "prims": prims[:50],
            "message": "Connected to Isaac Sim MCP"
        }
    }

def get_prim_info(params):
    """Get detailed info about a specific prim"""
    prim_path = params.get("prim_path", "")
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)

    if not prim:
        return {"status": "error", "message": f"Prim not found: {prim_path}"}

    # Get children
    children = [str(c.GetPath()) for c in prim.GetChildren()]

    # Get type
    prim_type = prim.GetTypeName()

    # Check for physics APIs
    has_rigid_body = prim.HasAPI(UsdPhysics.RigidBodyAPI)
    has_collision = prim.HasAPI(UsdPhysics.CollisionAPI)
    has_articulation = prim.HasAPI(UsdPhysics.ArticulationRootAPI)

    # Get bounding box
    bbox = None
    try:
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
        bound = cache.ComputeWorldBound(prim)
        if not bound.GetRange().IsEmpty():
            mn = bound.GetRange().GetMin()
            mx = bound.GetRange().GetMax()
            bbox = {
                "min": [mn[0], mn[1], mn[2]],
                "max": [mx[0], mx[1], mx[2]],
                "height": mx[2] - mn[2],
                "width": mx[0] - mn[0],
                "depth": mx[1] - mn[1]
            }
    except:
        pass

    return {
        "status": "success",
        "result": {
            "path": prim_path,
            "type": prim_type,
            "children": children[:20],
            "child_count": len(children),
            "has_rigid_body": has_rigid_body,
            "has_collision": has_collision,
            "has_articulation": has_articulation,
            "bbox": bbox
        }
    }

def list_children(params):
    """List all children of a prim with their types"""
    prim_path = params.get("prim_path", "/World")
    depth = params.get("depth", 1)
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)

    if not prim:
        return {"status": "error", "message": f"Prim not found: {prim_path}"}

    results = []
    for child in Usd.PrimRange(prim):
        child_path = str(child.GetPath())
        rel_depth = child_path.replace(prim_path, "").count("/")
        if rel_depth <= depth and rel_depth > 0:
            results.append({
                "path": child_path,
                "name": child_path.split("/")[-1],
                "type": child.GetTypeName()
            })

    return {
        "status": "success",
        "result": {
            "parent": prim_path,
            "children": results[:100]
        }
    }

def set_transform(params):
    """Set translation, rotation, or scale on a prim"""
    prim_path = params.get("prim_path", "")
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)

    if not prim:
        return {"status": "error", "message": f"Prim not found: {prim_path}"}

    xform = UsdGeom.Xformable(prim)

    # Clear existing ops if requested
    if params.get("clear", False):
        xform.ClearXformOpOrder()

    # Set translation
    translate = params.get("translate")
    if translate:
        xform.AddTranslateOp().Set(Gf.Vec3d(*translate))

    # Set rotation (euler XYZ)
    rotate = params.get("rotate")
    if rotate:
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotate))

    # Set scale
    scale = params.get("scale")
    if scale:
        if isinstance(scale, (int, float)):
            scale = [scale, scale, scale]
        xform.AddScaleOp().Set(Gf.Vec3f(*scale))

    return {"status": "success", "result": {"message": f"Transform set on {prim_path}"}}

def delete_prim(params):
    """Delete a prim from the stage"""
    prim_path = params.get("prim_path", "")
    stage = omni.usd.get_context().get_stage()

    if stage.GetPrimAtPath(prim_path):
        stage.RemovePrim(prim_path)
        return {"status": "success", "result": {"message": f"Deleted {prim_path}"}}
    else:
        return {"status": "error", "message": f"Prim not found: {prim_path}"}

def execute_script(code):
    """Execute Python code and capture all output"""
    import io
    from contextlib import redirect_stdout, redirect_stderr

    try:
        # Capture stdout and stderr
        stdout_capture = io.StringIO()
        stderr_capture = io.StringIO()

        local_ns = {
            "omni": omni,
            "Usd": Usd,
            "UsdGeom": UsdGeom,
            "UsdPhysics": UsdPhysics,
            "Sdf": Sdf,
            "Gf": Gf,
        }

        with redirect_stdout(stdout_capture), redirect_stderr(stderr_capture):
            exec(code, local_ns)

        stdout_text = stdout_capture.getvalue()
        stderr_text = stderr_capture.getvalue()

        # Also print to actual console for debugging
        if stdout_text:
            print(f"[Script output]: {stdout_text}")
        if stderr_text:
            print(f"[Script stderr]: {stderr_text}")

        return {
            "status": "success",
            "result": {
                "message": "Script executed",
                "stdout": stdout_text,
                "stderr": stderr_text
            }
        }
    except Exception as e:
        error_msg = traceback.format_exc()
        print(f"[Script error]: {error_msg}")
        return {"status": "error", "message": str(e), "traceback": error_msg}

def create_physics_scene(params):
    try:
        stage = omni.usd.get_context().get_stage()

        # Create World xform
        if not stage.GetPrimAtPath("/World"):
            UsdGeom.Xform.Define(stage, "/World")

        # Create physics scene
        if not stage.GetPrimAtPath("/World/PhysicsScene"):
            UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
            print("Created /World/PhysicsScene")

        # Create ground if requested
        if params.get("floor", True):
            if not stage.GetPrimAtPath("/World/GroundPlane"):
                # Create a large plane for ground
                ground = UsdGeom.Mesh.Define(stage, "/World/GroundPlane")
                ground.CreatePointsAttr([(-50, -50, 0), (50, -50, 0), (50, 50, 0), (-50, 50, 0)])
                ground.CreateFaceVertexCountsAttr([4])
                ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
                ground.CreateExtentAttr([(-50, -50, 0), (50, 50, 0)])

                # Add collision
                ground_prim = stage.GetPrimAtPath("/World/GroundPlane")
                UsdPhysics.CollisionAPI.Apply(ground_prim)
                print("Created /World/GroundPlane with collision")

        return {"status": "success", "result": {"message": "Physics scene created"}}
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}

def create_prim(params):
    try:
        prim_path = params.get("prim_path", "/World/NewPrim")
        prim_type = params.get("prim_type", "Xform")
        stage = omni.usd.get_context().get_stage()

        if prim_type == "Xform":
            UsdGeom.Xform.Define(stage, prim_path)
        elif prim_type == "Cube":
            UsdGeom.Cube.Define(stage, prim_path)
        elif prim_type == "Sphere":
            UsdGeom.Sphere.Define(stage, prim_path)
        else:
            UsdGeom.Xform.Define(stage, prim_path)

        return {"status": "success", "result": {"message": f"Created {prim_type} at {prim_path}"}}
    except Exception as e:
        return {"status": "error", "message": str(e)}

def load_usd(params):
    try:
        usd_path = params.get("path", "")
        prim_path = params.get("prim_path", "/World/Model")

        stage = omni.usd.get_context().get_stage()

        # Create parent if needed
        parent = "/".join(prim_path.split("/")[:-1])
        if parent and not stage.GetPrimAtPath(parent):
            UsdGeom.Xform.Define(stage, parent)

        # Add reference
        prim = stage.DefinePrim(prim_path)
        prim.GetReferences().AddReference(usd_path)

        print(f"Loaded {usd_path} at {prim_path}")
        return {"status": "success", "result": {"message": f"Loaded USD at {prim_path}"}}
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}

def add_physics(params):
    try:
        prim_path = params.get("prim_path", "")
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)

        if not prim:
            return {"status": "error", "message": f"Prim not found: {prim_path}"}

        # Add rigid body
        if params.get("rigid_body", True):
            rb = UsdPhysics.RigidBodyAPI.Apply(prim)
            rb.CreateRigidBodyEnabledAttr(True)
            rb.CreateKinematicEnabledAttr(False)
            print(f"Added RigidBodyAPI to {prim_path}")

        # Add collision to all mesh children
        if params.get("collision", True):
            count = 0
            for child in Usd.PrimRange(prim):
                if child.IsA(UsdGeom.Mesh):
                    if not child.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI.Apply(child)
                        count += 1
            print(f"Added CollisionAPI to {count} meshes under {prim_path}")

        # Set mass
        mass = params.get("mass", None)
        if mass:
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr(mass)
            print(f"Set mass to {mass} on {prim_path}")

        return {"status": "success", "result": {"message": f"Physics added to {prim_path}"}}
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}

# Global storage for articulations
_articulations = {}

def create_articulation(params):
    """Create an Articulation wrapper for a robot"""
    try:
        from omni.isaac.core.articulations import Articulation
        from omni.isaac.core.world import World

        prim_path = params.get("prim_path", "/World/Robot")
        name = params.get("name", "robot")

        # Get or create world
        global _articulations

        # Create articulation
        articulation = Articulation(prim_path=prim_path, name=name)

        # Store for later use
        _articulations[name] = articulation

        # Initialize if world exists
        try:
            articulation.initialize()
        except:
            pass

        # Get joint info
        dof_names = articulation.dof_names if articulation.dof_names else []
        num_dof = articulation.num_dof if articulation.num_dof else 0

        return {
            "status": "success",
            "result": {
                "message": f"Created articulation '{name}' at {prim_path}",
                "dof_names": list(dof_names) if dof_names else [],
                "num_dof": num_dof
            }
        }
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}

def get_joint_info(params):
    """Get joint names and current positions for an articulation"""
    try:
        from omni.isaac.core.articulations import Articulation

        prim_path = params.get("prim_path", "/World/Robot")

        articulation = Articulation(prim_path=prim_path)
        articulation.initialize()

        dof_names = list(articulation.dof_names) if articulation.dof_names is not None else []

        # Get current positions
        positions = articulation.get_joint_positions()
        pos_list = positions.tolist() if positions is not None else []

        # Get default state
        default_state = articulation.get_joints_default_state()
        default_pos = default_state.positions.tolist() if default_state and default_state.positions is not None else []

        return {
            "status": "success",
            "result": {
                "prim_path": prim_path,
                "dof_names": dof_names,
                "num_dof": len(dof_names),
                "current_positions": pos_list,
                "default_positions": default_pos
            }
        }
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}

def set_joint_states(params):
    """Directly set joint positions for an articulation"""
    try:
        from omni.isaac.core.articulations import Articulation
        import numpy as np

        prim_path = params.get("prim_path", "/World/Robot")
        positions = params.get("positions", {})  # dict of joint_name: position

        articulation = Articulation(prim_path=prim_path)
        articulation.initialize()

        dof_names = list(articulation.dof_names) if articulation.dof_names is not None else []

        # Build position array
        current_pos = articulation.get_joint_positions()
        if current_pos is None:
            current_pos = np.zeros(len(dof_names))
        else:
            current_pos = current_pos.copy()

        # Update positions from dict
        for name, pos in positions.items():
            if name in dof_names:
                idx = dof_names.index(name)
                current_pos[idx] = pos
                print(f"Setting {name} to {pos}")

        # Set positions
        articulation.set_joint_positions(current_pos)

        # Also set as default state
        articulation.set_joints_default_state(positions=current_pos)

        return {
            "status": "success",
            "result": {
                "message": f"Set joint positions on {prim_path}",
                "positions_set": list(positions.keys())
            }
        }
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}

def step_physics(params):
    """Step the physics simulation"""
    try:
        import omni.physx
        from omni.physx import get_physx_interface

        steps = params.get("steps", 1)

        physx = get_physx_interface()
        for _ in range(steps):
            physx.update_simulation(1.0/60.0, 1.0/60.0)

        return {
            "status": "success",
            "result": {"message": f"Stepped physics {steps} times"}
        }
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}

def take_screenshot(params):
    """Capture a screenshot of the viewport"""
    try:
        import omni.kit.viewport.utility as viewport_util
        from PIL import Image
        import numpy as np
        import base64
        import io

        output_path = params.get("path", "D:/Projects/ATST/isaac_sim/screenshot.png")

        # Get the active viewport
        viewport = viewport_util.get_active_viewport()
        if not viewport:
            return {"status": "error", "message": "No active viewport"}

        # Capture the viewport
        capture = viewport_util.capture_viewport_to_file(viewport, output_path)

        print(f"Screenshot saved to: {output_path}")
        return {
            "status": "success",
            "result": {
                "message": f"Screenshot saved to {output_path}",
                "path": output_path
            }
        }
    except ImportError as e:
        # Fallback method using carb
        try:
            import carb.settings
            import omni.renderer_capture

            output_path = params.get("path", "D:/Projects/ATST/isaac_sim/screenshot.png")

            # Use renderer capture
            capture_interface = omni.renderer_capture.acquire_renderer_capture_interface()
            capture_interface.capture_next_frame_to_file(output_path)

            return {
                "status": "success",
                "result": {
                    "message": f"Screenshot queued to {output_path}",
                    "path": output_path
                }
            }
        except Exception as e2:
            return {"status": "error", "message": f"Screenshot failed: {e}, fallback: {e2}"}
    except Exception as e:
        traceback.print_exc()
        return {"status": "error", "message": str(e)}

# Start MCP server
mcp_server = MCPServer()
mcp_server.start()

print("=" * 60)
print("Isaac Sim running with MCP support")
print("MCP server listening on localhost:8766")
print("Close the window when done.")
print("=" * 60)
sys.stdout.flush()

# Main loop - process command queue here
try:
    while app.is_running():
        # Process any queued commands on main thread
        try:
            while not command_queue.empty():
                request_id, command = command_queue.get_nowait()
                print(f"Executing command: {command.get('type')}")
                response = execute_command(command)
                print(f"Command result: {response.get('status')}")
                if request_id in response_queues:
                    response_queues[request_id].put(response)
        except queue.Empty:
            pass

        app.update()
finally:
    mcp_server.stop()
    app.close()
