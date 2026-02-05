# AT-ST Project - TODO

## Current Priority
**Visual Model Assembly** - Split Sketchfab mesh by bones, export as USD, add physics articulation.

---

## Current Phase: Visual Model Integration

### Blender Work (using Blender MCP)
- [ ] Complete mesh splitting by bone weights
- [ ] Verify all body parts separated: head, body, hips, thighs, shins, ankles, feet
- [ ] Clean up any remaining unsplit geometry
- [ ] Set up proper armature hierarchy
- [ ] Export split model as USD (`D:/Projects/ATST/usd/atst_visual.usda`)

### Physics Integration
- [ ] Create articulated USD with visual meshes
- [ ] Map joint hierarchy to match trained physics model
- [ ] Apply collision meshes to visual parts
- [ ] Test visual model with trained walking policy

---

## Completed ✓

### Phase 1: Physics Model & RL Training (DONE)
- [x] Install Isaac Sim 5.1 via pip
- [x] Install PyTorch 2.7.0 with CUDA 12.8
- [x] Clone and install Isaac Lab
- [x] Clone isaac-sim-mcp for AI control
- [x] Create MCP server integration (localhost:8766)
- [x] AT-ST model loads and displays correctly
- [x] Create Start Menu shortcut ("Isaac")
- [x] Set up project documentation

### Phase 2: Digitigrade Walking (DONE)
- [x] Create digitigrade physics model (`create_atst_usd.py`)
- [x] Build 8-joint leg structure (hip_pitch, hip_roll, knee, ankle × 2)
- [x] Configure robot for Isaac Lab (`atst_robot_cfg.py`)
- [x] Set up RL environment (`atst_env_cfg.py`)
- [x] Train walking policy with RSL-RL PPO (`train_atst.py`)
- [x] Achieve ~82% survival rate
- [x] Create visualization script (`play_atst.py`)

### Phase 3: Visual Model Analysis (DONE)
- [x] Analyze Thingiverse 3D printable model (42 STL parts)
- [x] Analyze Sketchfab rigged model (31-bone skeleton)
- [x] Import FBX into Blender
- [x] Create mesh splitting scripts
- [x] Set up Blender MCP addon

---

## Future Phases

### Phase 4: Polish & Testing
- [ ] Fine-tune walking gait
- [ ] Add terrain variations
- [ ] Improve visual fidelity

### Phase 5: Advanced Features
- [ ] Rough terrain navigation
- [ ] Head/turret movement
- [ ] Weapon animations

---

## Notes
- Working model: `D:/Projects/ATST/logs/atst_training/2026-02-04_18-25-00/model_final.pt`
- Physics USD: `D:/Projects/ATST/usd/atst_rl.usda`
- Sketchfab bones: ATST, Body, head, Torso_L/R, Hip1-3_L/R, Thigh_L/R, Shin_L/R, Ankle_L/R, Foot_L/R
- Blender MCP configured: `claude mcp add blender -- uvx blender-mcp`
