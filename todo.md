# AT-ST Project - TODO

## Current Priority
**Visual Mesh Integration** - Convert OBJ meshes to USD format and attach to working physics model.

---

## Current Phase: Visual Model Integration

### Immediate Next Steps
- [ ] Convert OBJ parts to USD using Isaac Sim's asset converter
- [ ] Reference converted USD meshes in the working `atst_rl.usda` model
- [ ] Test visual model renders in Isaac Sim
- [ ] Verify physics still works with visual meshes attached

### Alternative Approach (if conversion fails)
- [ ] Create USD meshes directly in Blender (export as .usd instead of .obj)
- [ ] Use proper USD references in the physics model
- [ ] Test end-to-end

---

## Completed This Session ✓

### Phase 3.5: Visual Mesh Export (DONE)
- [x] Connect to Blender via MCP
- [x] Split AT-ST mesh into 10 articulated parts by bone weights
- [x] Export parts as individual OBJ files
- [x] Create URDF robot description
- [x] Diagnose mesh visibility issue (OBJ references not resolving)
- [x] Identify solution: use native USD geometry or convert OBJs

---

## Completed Previously ✓

### Phase 1: Physics Model & RL Training (DONE)
- [x] Install Isaac Sim 5.1 via pip
- [x] Create MCP server integration (localhost:8766)
- [x] Set up project documentation

### Phase 2: Digitigrade Walking (DONE)
- [x] Create digitigrade physics model (`create_atst_usd.py`)
- [x] Train walking policy with RSL-RL PPO
- [x] Achieve ~82% survival rate

### Phase 3: Visual Model Analysis (DONE)
- [x] Analyze Sketchfab rigged model (31-bone skeleton)
- [x] Set up Blender MCP addon

---

## Future Phases

### Phase 4: Polish & Testing
- [ ] Fine-tune walking gait with visual model
- [ ] Add terrain variations
- [ ] Improve visual fidelity (materials, textures)

### Phase 5: Advanced Features
- [ ] Rough terrain navigation
- [ ] Head/turret movement
- [ ] Weapon animations

---

## Suggested Follow-On Prompts

Use these prompts to continue the project in a new conversation:

### Option 1: Quick Test (Verify Physics Model Still Works)
```
I'm working on the AT-ST project. Can you help me run the existing working walker
to verify it still displays correctly? The command should be:
D:\Projects\ATST\venv_isaaclab\Scripts\python.exe D:\Projects\ATST\isaac_sim\play_atst.py
```

### Option 2: Convert OBJ to USD (Recommended Next Step)
```
I'm working on the AT-ST project. We have visual mesh parts exported as OBJ files
in D:\Projects\ATST\usd\parts\. Can you help me convert these to USD format using
Isaac Sim's asset converter, then reference them in the working physics model
(D:\Projects\ATST\usd\atst_rl.usda)?
```

### Option 3: Re-export from Blender as USD
```
I'm working on the AT-ST project. We have the split AT-ST model in Blender
(D:\Projects\ATST\models\atst_articulated.blend). Can you help me export the
parts directly as USD files instead of OBJ, which should work better with Isaac Sim?
```

### Option 4: Create Visual Overlay Script
```
I'm working on the AT-ST project. The physics model works great with primitive
geometry. Can you create a script that adds the detailed visual meshes as child
prims to each physics link, so we get the nice visuals while keeping the working
physics?
```

---

## Reference Files

| File | Description |
|------|-------------|
| `usd/atst_rl.usda` | Working physics model (primitives) |
| `usd/parts/*.obj` | Visual mesh parts (10 files) |
| `models/atst_articulated.blend` | Blender source file |
| `urdf/atst_robot.urdf` | Robot description |
| `logs/atst_training/.../model_final.pt` | Trained RL policy |
| `isaac_sim/play_atst.py` | Visualization script |
| `isaac_sim/add_visual_meshes.py` | Visual integration script |

---

## Key Insight from This Session

The working AT-ST uses **native USD geometry** (`def Cube`, `def Capsule`) which
renders automatically in Isaac Sim. External mesh file references (OBJ via URDF)
require conversion to USD format to render properly.
