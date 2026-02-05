# AT-ST / Cassie Walking Robot - Isaac Sim 5.1 Integration

Complete simulation and control setup for building a 1:1 scale bipedal/quadrupedal walking robot with Isaac Sim 5.1, ROS2, and real-world hardware integration.

## Project Structure

```
D:\Projects\ATST\
├── models/                    # Downloaded 3D model files (FBX, USD, textures)
├── usd/                       # Converted USD files for Isaac Sim
├── isaac_sim/                 # Isaac Sim scene files (.usda)
├── isaac_scripts/             # Python scripts for Isaac Sim simulation
├── cad/                       # CAD files for manufacturability (STEP, IGES)
├── manufacturing/             # G-code, STL files for 3D printing / CNC
├── sim_configs/               # ROS2 control YAML, controller configs
├── bom/                       # Bill of materials (actuators, sensors, power)
├── hardware/                  # Hardware integration (schematics, pinouts)
├── firmware/                  # Embedded control firmware (Arduino, etc.)
├── exports/                   # Generated exports (URDF, Meshes, etc.)
└── docs/                      # Documentation and references
```

## Prerequisites

### 1. Install NVIDIA Omniverse & Isaac Sim 5.1

```powershell
# Download Omniverse Launcher
# https://www.nvidia.com/en-us/omniverse/download/

# Launch Omniverse, sign in, go to "Exchange"
# Install "Isaac Sim" (latest version, 5.1+)

# Verify installation
C:\Users\<you>\AppData\Local\ov\pkg\isaac-sim-5.1.0\python.bat -c "import omni; print('Isaac Sim ready')"
```

### 2. Install ROS2 Humble (Windows)

```powershell
# Download ROS2 Humble for Windows
# https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html

# Or use WSL2 (recommended for easier ROS2 setup)
wsl --install Ubuntu-22.04
```

### 3. Install Dependencies

```powershell
# Python packages for Isaac Sim scripts
pip install numpy scipy pyyaml rclpy

# Or use Isaac Sim's included Python
python.bat -m pip install numpy scipy pyyaml rclpy
```

## Quick Start: Running the Cassie Walking Simulation

### Option 1: Simple Walking Scene (No ROS2)

```powershell
cd D:\Projects\ATST

# Run using Isaac Sim's Python environment
C:\Users\<you>\AppData\Local\ov\pkg\isaac-sim-5.1.0\python.bat isaac_scripts\cassie_scene.py
```

**Expected Output:**
- Isaac Sim window opens with Cassie robot
- Robot performs sinusoidal walking gait
- Console prints iteration progress and velocities
- Simulation runs for ~50 seconds

### Option 2: Walking with ROS2 Control (Full Integration)

**Terminal 1: Start ROS2 Daemon**
```powershell
# If using WSL2 ROS2:
wsl
source /opt/ros/humble/setup.bash
ros2 daemon start
```

**Terminal 2: Run Cassie Simulation**
```powershell
cd D:\Projects\ATST
C:\Users\<you>\AppData\Local\ov\pkg\isaac-sim-5.1.0\python.bat isaac_scripts\cassie_ros2_bridge.py
```

**Terminal 3 (WSL2): Monitor ROS2 Topics**
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /cassie/odometry
```

## Scripts Overview

### `cassie_scene.py`
- **Purpose:** Loads Cassie robot, configures physics/articulation, runs walking gait
- **Features:**
  - Full PhysX articulation with 8 DOF bipedal legs
  - Sinusoidal gait controller (1.0s cycle time)
  - Joint damping and effort limits configured
  - Real-time rendering with physics feedback
- **Usage:** `python cassie_scene.py`

### `cassie_ros2_bridge.py`
- **Purpose:** Real-time ROS2 integration for Isaac Sim
- **Topics:**
  - **Publish:** `/joint_states` (sensor_msgs/JointState), `/cassie/odometry` (geometry_msgs/Twist)
  - **Subscribe:** `/cassie/joint_commands` (std_msgs/Float64MultiArray)
- **Usage:** Run in background; publish joint commands from ROS2 control stack
- **Example ROS2 Command:**
  ```bash
  ros2 topic pub /cassie/joint_commands std_msgs/Float64MultiArray "{data: [0.1, 0.3, -0.5, 0.5, 0.1, 0.3, -0.5, 0.5]}"
  ```

## Customization & Extension

### Change Walking Gait Parameters

Edit `CassieGaitController` in `cassie_scene.py`:

```python
self.cycle_time = 0.8  # Speed up walking (0.8s cycle)

self.amplitude = np.array([
    0.15, 0.4, 0.4, 0.25,  # Increase step height
    0.15, 0.4, 0.4, 0.25,
])
```

### Switch from Cassie to AT-ST

1. Convert AT-ST FBX to USD:
   ```
   1. Open D:\Projects\ATST\models\atst_model.fbx in Blender
   2. File → Export → Universal Scene Description (.usda)
   3. Save to D:\Projects\ATST\usd\atst_model.usda
   ```

2. Update robot path in `cassie_scene.py`:
   ```python
   atst_usd = "D:\\Projects\\ATST\\usd\\atst_model.usda"
   atst = world.scene.add(Robot(prim_path="/World/ATST", usd_path=atst_usd, ...))
   ```

3. Adjust gait for quadruped (4 legs, 2 DOF each = 8 DOF):
   ```python
   # Map to front-left, front-right, rear-left, rear-right
   amplitude = np.array([
       0.2, 0.3,  # Front-left
       0.2, 0.3,  # Front-right
       0.2, 0.3,  # Rear-left
       0.2, 0.3,  # Rear-right
   ])
   ```

### Add Sensors

```python
# In cassie_scene.py, after loading robot:
from omni.isaac.sensor import IMUSensor

imu = IMUSensor(prim_path="/World/Cassie/pelvis/imu_sensor")
```

### Real-Time Control (Custom ROS2 Node)

```bash
ros2 node create cassie_controller --parameters sim_configs/cassie_controllers.yaml
ros2 control load-controller cassie_joint_trajectory_controller
ros2 control set-active cassie_joint_trajectory_controller
```

## Physics Tuning

Adjust in `cassie_scene.py`:

```python
physics_context.set_solver_type("TGS")      # TGS or PGS
physics_context.set_broadphase_type("GPU")  # GPU for faster collision
physics_context.set_gravity([0, 0, -9.81])  # Gravity magnitude
```

## Performance Tips

1. **Headless Mode** (faster simulation):
   ```python
   SimulationApp({"headless": True})
   ```

2. **GPU Physics** (if NVIDIA GPU available):
   ```python
   physics_context.enable_gpu_dynamics(True)
   ```

3. **Reduce Rendering** (update render every N steps):
   ```python
   if iteration % 10 == 0:
       world.step(render=True)
   else:
       world.step(render=False)
   ```

## Next Steps: Hardware Deployment

1. **URDF Export:** Use `omni.isaac.kit.xacro` to export URDF for real robot
2. **Actuator Selection:** Choose motors, gearboxes, and controllers (see `bom/`)
3. **Control Firmware:** Deploy ros2_control stack on embedded system (Jetson, etc.)
4. **Hardware Testing:** Validate sensor fusion, gait tuning, and safety limits

## Troubleshooting

### "ModuleNotFoundError: No module named 'omni'"
- Ensure you're running with Isaac Sim's Python:
  ```powershell
  C:\Users\<you>\AppData\Local\ov\pkg\isaac-sim-5.1.0\python.bat script.py
  ```

### "ROS2 topics not appearing"
- Verify ROS2 is installed and sourced
- Use `ros2 daemon start` and `ros2 topic list` to check connectivity

### Cassie falls over or unstable
- Increase joint damping (kds in set_joint_gains)
- Decrease cycle_time (slow down walking)
- Raise initial position (start higher above ground)

## References

- [NVIDIA Isaac Sim Docs](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [Cassie Robot](https://agility-robotics.com/)

## License

AT-ST model: CC BY 4.0 (Star Wars AT-ST [Rigged] by smithson17)
Simulation code: MIT

---

**Last Updated:** February 3, 2026  
**Isaac Sim Version:** 5.1+  
**ROS2 Version:** Humble
