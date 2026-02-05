# Cassie using Isaac Lab's Articulation API and proper joint state setting
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

# Fresh stage
print("Creating fresh stage...")
send_command("execute_script", {"code": '''
import omni.usd
omni.usd.get_context().new_stage()
import omni.kit.app
omni.kit.app.get_app().update()
'''})
time.sleep(2)

# Setup using Isaac Lab's approach
script = '''
import torch
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf, UsdLux

# Import Isaac Lab components
try:
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab.sim import SimulationContext
    from isaaclab_assets.robots.cassie import CASSIE_CFG
    print("Isaac Lab imports successful!")
    USE_ISAACLAB = True
except Exception as e:
    print(f"Isaac Lab not available: {e}")
    USE_ISAACLAB = False

stage = omni.usd.get_context().get_stage()

if USE_ISAACLAB:
    print("\\nSetting up with Isaac Lab...")

    # Create simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.005)
    sim = SimulationContext(sim_cfg)

    # Ground plane
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/defaultGroundPlane", ground_cfg)

    # Lights
    light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)

    # Create Cassie with proper config
    print("Loading Cassie with CASSIE_CFG...")
    cassie = Articulation(CASSIE_CFG.replace(prim_path="/World/Cassie"))

    # Reset simulation to initialize everything
    sim.reset()

    # Write the default joint state directly to simulation
    joint_pos = cassie.data.default_joint_pos
    joint_vel = cassie.data.default_joint_vel
    cassie.write_joint_state_to_sim(joint_pos, joint_vel)

    # Write root pose
    root_state = cassie.data.default_root_state.clone()
    cassie.write_root_pose_to_sim(root_state[:, :7])
    cassie.write_root_velocity_to_sim(root_state[:, 7:])
    cassie.reset()

    print("\\nCassie initialized with Isaac Lab!")
    print(f"Default joint positions: {joint_pos}")

    # Set camera
    sim.set_camera_view(eye=[3.0, -3.0, 2.0], target=[0.0, 0.0, 0.9])

    print("\\n=== Cassie ready with Isaac Lab ===")
    print("Press PLAY in Isaac Sim!")

else:
    # Fallback - manual setup
    print("\\nFalling back to manual setup...")

    # Basic scene
    UsdGeom.Xform.Define(stage, "/World")

    # Ground
    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(10)
    ground.CreateDisplayColorAttr([(0.3, 0.3, 0.3)])
    gprim = stage.GetPrimAtPath("/World/Ground")
    UsdGeom.Xformable(gprim).AddTranslateOp().Set(Gf.Vec3d(0, 0, -5))
    UsdPhysics.CollisionAPI.Apply(gprim)

    # Physics scene
    physicsScene = UsdPhysics.Scene.Define(stage, "/physicsScene")
    physicsScene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physicsScene.CreateGravityMagnitudeAttr(9.81)

    # Light
    UsdLux.DistantLight.Define(stage, "/World/Light").CreateIntensityAttr(3000)

    # Load Cassie
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.nucleus import get_assets_root_path

    assets_root = get_assets_root_path()
    cassie_usd = assets_root + "/Isaac/Robots/Agility/Cassie/cassie.usd"
    add_reference_to_stage(usd_path=cassie_usd, prim_path="/World/Cassie")

    import omni.kit.app
    for i in range(10):
        omni.kit.app.get_app().update()

    # Position
    cassie = stage.GetPrimAtPath("/World/Cassie")
    if cassie:
        xf = UsdGeom.Xformable(cassie)
        for op in xf.GetOrderedXformOps():
            if "translate" in op.GetOpName():
                op.Set(Gf.Vec3d(0, 0, 0.9))
                break

    print("Manual setup complete - press PLAY")
'''

print("Setting up Cassie with Isaac Lab style...")
result = send_command("execute_script", {"code": script})
if result.get("status") == "success":
    stdout = result.get("result", {}).get("stdout", "")
    if stdout:
        print(stdout)
else:
    print(f"Error: {result.get('message')}")

time.sleep(2)
result = send_command("screenshot", {"path": "D:/Projects/ATST/isaac_sim/screenshot.png"})
print("\nScreenshot taken")
