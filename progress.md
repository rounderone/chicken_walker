# AT-ST Project - Progress Log

## Session: 2026-02-05 (Current)

### Focus: Visual Model Integration via Blender MCP

### Accomplishments

1. **Used Blender MCP to Split AT-ST Mesh**
   - Connected to Blender via MCP server (port 9876)
   - Split Sketchfab AT-ST model into 10 articulated parts by bone weights
   - Parts: torso, head, thigh_L/R, shin_L/R, elbow_L/R, foot_L/R
   - Saved as: `D:/Projects/ATST/models/atst_articulated.blend`

2. **Exported Parts as OBJ Files**
   - Exported each part to `D:/Projects/ATST/usd/parts/`
   - Files: ATST_torso.obj, ATST_head.obj, ATST_thigh_L/R.obj, etc.

3. **Created URDF Robot Description**
   - Built `D:/Projects/ATST/urdf/atst_robot.urdf`
   - 10 links with visual mesh references
   - 9 revolute joints with proper limits
   - Mass properties: heavy feet (500kg) for stability

4. **Diagnosed Mesh Visibility Issue**
   - URDF-imported OBJ meshes not rendering in Isaac Sim
   - **Root cause**: External OBJ file references not resolving properly
   - **Discovery**: Working AT-ST uses native USD geometry (Cube/Capsule primitives)
   - These render automatically without external file dependencies

5. **Created Visual Mesh Integration Script**
   - `isaac_sim/add_visual_meshes.py` - loads working physics model and adds visuals
   - Approach: Keep working primitive physics, overlay detailed meshes

### Key Insight
The working `atst_rl.usda` model succeeds because it uses **native USD geometry types** (`def Cube`, `def Capsule`) which Isaac Sim renders automatically. URDF imports with external mesh file references require additional conversion steps.

### Files Created This Session
```
urdf/
  atst_robot.urdf              # Robot description with OBJ mesh references

usd/parts/
  ATST_torso.obj               # Individual mesh parts from Blender
  ATST_head.obj
  ATST_thigh_L.obj, ATST_thigh_R.obj
  ATST_shin_L.obj, ATST_shin_R.obj
  ATST_elbow_L.obj, ATST_elbow_R.obj
  ATST_foot_L.obj, ATST_foot_R.obj

models/
  atst_articulated.blend       # Blender file with split/rigged AT-ST

isaac_sim/
  add_visual_meshes.py         # Script to add visual meshes to physics model
```

### Current State
- **Physics walker**: WORKING - primitive geometry with trained policy
- **Visual meshes**: EXPORTED - 10 OBJ files ready for integration
- **Integration**: IN PROGRESS - need to convert OBJs to USD or use asset converter

---

## Session: 2026-02-04

### Major Accomplishment: WORKING WALKER!
Successfully trained an RL policy that makes a digitigrade AT-ST walk! The physics model uses primitive geometry (capsules/boxes) and achieves ~82% survival rate.

### Accomplishments

1. **Created Digitigrade Physics Model**
   - Built `create_atst_usd.py` - generates USD with 8 joints (4 per leg)
   - Joint structure: hip_pitch → hip_roll → knee → ankle
   - Shins angled 45° backward for "chicken walker" look
   - Output: `D:/Projects/ATST/usd/atst_rl.usda`

2. **Configured Robot for Isaac Lab**
   - `atst_robot_cfg.py` - ArticulationCfg with actuator groups
   - `atst_env_cfg.py` - Full RL environment config with rewards/terminations
   - Uses locomotion-specific biped rewards from Isaac Lab

3. **Trained Walking Policy**
   - `train_atst.py` - RSL-RL PPO training script
   - 4096 parallel environments, trained to convergence
   - Model saved: `D:/Projects/ATST/logs/atst_training/2026-02-04_18-25-00/model_final.pt`
   - `play_atst.py` - Visualization script (currently running at 119k+ steps!)

4. **Visual Model Progress**
   - Analyzed Thingiverse 3D printable AT-ST (42 STL parts, 14 joints)
   - Analyzed Sketchfab rigged model (has 31-bone skeleton)
   - Imported `ATST workshop.fbx` into Blender
   - Created mesh splitting script `import_and_split_atst.py`
   - Partial split achieved - need to complete leg separation

5. **Set Up Blender MCP**
   - Downloaded blender-mcp addon to `D:/Projects/ATST/blender-mcp/addon.py`
   - Added MCP server to Claude: `claude mcp add blender -- uvx blender-mcp`
   - Ready to connect after restart

### Current State
- **Physics walker**: WORKING - primitive geometry, trained policy walking
- **Visual model**: IN PROGRESS - Sketchfab model in Blender, needs mesh splitting by bones
- **Blender MCP**: CONFIGURED - need restart to activate

### Files Created This Session
```
isaac_sim/
  create_atst_usd.py      # Generates physics USD model
  atst_robot_cfg.py       # Robot articulation config
  atst_env_cfg.py         # RL environment config
  train_atst.py           # Training script
  play_atst.py            # Visualization script
  inspect_atst_model.py   # Model inspection utility
  reanalyze_sketchfab.py  # Skeleton analysis
  analyze_stl_model.py    # STL dimension analysis

blender/
  assemble_atst.py              # Import Thingiverse STLs
  import_and_split_atst.py      # Import FBX and split by bones
  split_sketchfab_by_bones.py   # Mesh splitting utility
  check_vertex_groups.py        # Debug script
  atst_split.blend              # Partially split model

blender-mcp/
  addon.py                # Blender MCP addon for Claude control

logs/atst_training/
  2026-02-04_18-25-00/    # Trained model checkpoint
```

---

## Session: 2026-02-02

### Summary
Set up Isaac Sim 5.1 via pip, created MCP server for AI control, prepared for physics testing.

### Accomplishments
1. Cleaned up old Isaac Sim installation (freed ~50-100GB)
2. Installed Isaac Sim 5.1 via pip in venv
3. Created MCP server for AI control (localhost:8766)
4. Set up project infrastructure (launcher, shortcuts, docs)

---

## Previous Sessions

### Initial Setup
- Obtained AT-ST model from Sketchfab
- Converted to USD via Blender
- Model saved at `D:\Projects\ATST\usd\atst.usdc`

---

## Reference: What Works
- Digitigrade physics model with 8 joints walks successfully
- RSL-RL PPO training with Isaac Lab
- 4096 parallel environments for fast training
- Sketchfab model has proper bone skeleton (31 bones)
- Thingiverse model has separated parts (42 STL files)

## Reference: Key Dimensions (Physics Model)
- Total height: ~0.75m
- Body at z=0.74m, Hip at z=0.57m
- Shin angle: 45° backward
- Leg spacing: 0.40m
- Foot: 0.18m x 0.12m x 0.03m
