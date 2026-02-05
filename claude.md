# Claude Instructions for AT-ST Project

## Project Overview
**Goal:** Build a functional, physics-based AT-ST (Star Wars chicken walker) in Isaac Sim 5.1 that stands stably and eventually walks with articulated reverse-knee legs. Use heavy feet for balance to mimic digitigrade locomotion (inspired by ostriches/Cassie robot).

**User Level:** Complete beginner—no coding or 3D experience. Keep things simple and well-explained.

## Key Rules

### File Management
- **USE EXISTING FILES** - Don't create new files if one already exists for that purpose
- **DON'T RENAME** files or variables without explicit permission
- **DON'T CREATE SIMILAR PROGRAMS** - If `setup_atst.py` exists, don't make `setup_atst_v2.py`
- **CONSOLIDATE** - If multiple scripts do similar things, combine them into one
- **BACKUP MILESTONES** - Only create backups when user requests or at major milestones

### Documentation
- **ONLY THREE MD FILES**: `claude.md`, `todo.md`, `progress.md`
- Don't create additional documentation files without permission
- Update these files to maintain context between conversations

### Scripting Preferences
- Generate **standalone Python scripts** (GUI steps only when simpler)
- Scripts run via: `D:\Projects\ATST\venv_isaaclab\Scripts\python.exe script.py`
- Or via launcher: `D:\Projects\ATST\isaac_sim\launch_isaac.bat`
- Include SimulationApp launch with `headless: False` for visibility
- Lots of comments, print statements with `sys.stdout.flush()`, error handling
- Save scenes as new .usda files in `D:\Projects\ATST\usd\`
- After script: Give exact CLI command + any GUI steps needed

### Communication Style
- Always ask for feedback/screenshots after each script
- Keep responses encouraging and step-by-step
- When things go wrong, offer both quick fixes and explanations

## Key Paths
```
D:\Projects\ATST\              # Project root
├── isaac_sim\                 # Python scripts
│   ├── launch_isaac.bat       # Main launcher (Start Menu: "Isaac")
│   ├── launch_isaac_mcp.py    # Isaac Sim with MCP server
│   └── setup_atst.py          # AT-ST setup script
├── usd\                       # USD model files
│   ├── atst.usdc              # Main AT-ST model (Sketchfab rigged version)
│   └── configuration\         # Model config files
├── IsaacLab\                  # Isaac Lab (cloned repo)
├── isaac-sim-mcp\             # MCP integration (cloned repo)
├── venv_isaaclab\             # Python venv with Isaac Sim 5.1 + PyTorch
├── claude.md                  # These instructions
├── todo.md                    # Current tasks
└── progress.md                # Session logs
```

## Common Commands
```bash
# Launch Isaac Sim with MCP server
D:\Projects\ATST\isaac_sim\launch_isaac.bat

# Or from Start Menu - type "Isaac"

# Run a script directly
D:\Projects\ATST\venv_isaaclab\Scripts\python.exe D:\Projects\ATST\isaac_sim\script.py

# Quick Python (system-wide 3.11)
py -3.11 -c "print('hello')"

# Test MCP connection (when Isaac Sim running)
py -3.11 -c "import socket,json;s=socket.socket();s.connect(('localhost',8766));s.sendall(json.dumps({'type':'get_scene_info','params':{}}).encode());s.settimeout(5);print(s.recv(16384).decode());s.close()"
```

## Troubleshooting Reference
- **Payload errors:** Avoid references—open stage directly or merge/flatten
- **Model not visible:** Turn on "Camera Light" (top toolbar), Frame with F key
- **Falls through floor:** Need CollisionAPI on BOTH ground AND model meshes
- **Physics not responding:** Check kinematicEnabled=False, need RigidBodyAPI
- **Unstable/tips over:** Add heavy feet (500-1000kg each) to lower center of mass
- **Foot prim names:** Usually contain "foot", "toe", "pad"—check Stage panel
- **Isaac Sim freezes:** Give it time to warm up, or restart if truly frozen

## MCP Server (AI Control)
When Isaac Sim runs via `launch_isaac.bat`, MCP server is on `localhost:8766`
- Working: `get_scene_info`, `create_prim`, `create_physics_scene`
- Issues: `execute_script` can freeze Isaac Sim (threading problem)
- Fallback: Use manual GUI or standalone scripts when MCP fails
