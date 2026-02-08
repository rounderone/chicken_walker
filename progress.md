# AT-ST Project - Progress Log

## Session: 2026-02-08 (Current)

### MAJOR MILESTONE: Deep Crouch with Articulated Hip Working!

### Accomplishments

1. **Separated Bar Geometry from Thigh in Blender**
   - Used Blender MCP to select vertices by Y position (Y > 0.6)
   - Separated transfer bar mesh from thigh mesh for each leg
   - Set bar origins to geometry centers for proper pivot points
   - Bar origins: bar_L at (1.026, 1.788, -1.219), bar_R mirrored

2. **Exported 12-Part Articulated Model**
   - Exported `D:/Projects/ATST/usd/atst_with_bars.usda`
   - Parts: torso, head, bar_L, bar_R, thigh_L, thigh_R, shin_L, shin_R, elbow_L, elbow_R, foot_L, foot_R
   - Model coordinates with Z-offset of 4.05 to put feet on ground

3. **User Verified Joint Positions**
   - Joint markers saved in `D:/Projects/ATST/usd/202602072308_joints.usda`
   - Critical positions (model coords, add 4.05 for world Z):
     - neck: (0.0, 0.263, -0.669)
     - torsobar_L: (0.640, 1.050, -1.043) - Torso to bar lateral joint
     - legbar_L: (0.727, 0.449, -1.031) - Bar to thigh (hip rotation)
     - knee_L: (0.944, 1.861, -1.230)
     - ankle_L: (0.776, 1.515, -3.452)
     - foot_L: (0.757, 1.010, -3.697)

4. **Parallel Testing of 9 Robot Configurations**
   - Test script: `isaac_sim/test_nine_behaviors.py`
   - Tested knee angles K40-K80 with hip ratios 0.3-0.7
   - **WINNER: Robot 0 (K60 H+18)** - Only config that stayed standing during deep crouch
   - Working parameters: knee_max=60, hip_ratio=0.3

5. **Created Checkpoint Files**
   - `checkpoints/checkpoint_20260208_deep_crouch_working.usda` - Model
   - `checkpoints/checkpoint_20260208_verified_joints.usda` - Joint markers
   - `checkpoints/checkpoint_20260208_test_script.py` - Working test script

### Key Technical Details

**Joint Hierarchy:**
```
torso (ArticulationRoot)
  -> bar_L/R (Y-axis lateral swivel)
     -> thigh_L/R (X-axis hip rotation)
        -> shin_L/R (X-axis knee)
           -> elbow_L/R (X-axis)
              -> foot_L/R (X-axis ankle)
```

**Working Crouch Parameters:**
```python
knee_max = 60       # Maximum knee bend in degrees
hip_ratio = 0.3     # Hip compensation = knee_angle * 0.3
# So at full crouch: knee=-60, hip=+18
```

**Part Masses (for stability):**
```python
PART_MASSES = {
    'ATST_torso': 20.0, 'ATST_head': 10.0,
    'ATST_bar_L': 2.0, 'ATST_bar_R': 2.0,
    'ATST_thigh_L': 4.0, 'ATST_thigh_R': 4.0,
    'ATST_shin_L': 4.0, 'ATST_shin_R': 4.0,
    'ATST_elbow_L': 3.0, 'ATST_elbow_R': 3.0,
    'ATST_foot_L': 15.0, 'ATST_foot_R': 15.0,
}
```

**No-Collision Parts (to prevent self-collision):**
```python
NO_COLLISION_PARTS = {'ATST_thigh_L', 'ATST_thigh_R', 'ATST_bar_L', 'ATST_bar_R'}
```

---

## Session: 2026-02-07

### Focus: Hip Joint Architecture

1. Discovered the AT-ST uses a compound hip mechanism:
   - Transfer bar connects torso to thigh
   - Bar swivels laterally (Y-axis) at torso connection
   - Hip rotation (X-axis) occurs at bar-to-thigh connection

2. This matches real AT-ST design from Star Wars reference materials

---

## Key Files (Golden Path)

| File | Description |
|------|-------------|
| `usd/atst_with_bars.usda` | Working 12-part articulated model |
| `usd/202602072308_joints.usda` | User-verified joint marker positions |
| `isaac_sim/test_nine_behaviors.py` | Working physics test script |
| `checkpoints/` | Milestone backups |

## What Works

- 12-part articulated model with separated bar geometry
- Deep crouch (60 degree knee) with hip compensation (0.3 ratio)
- Robot stays level during crouch using bar lateral + hip rotation
- Convex hull collision on feet, shins, elbows
- No collision on thighs/bars to prevent self-intersection

## Next Steps

- Fine-tune deeper crouch (K70, K80) by adjusting hip ratio
- Add walking gait while maintaining stable crouch mechanics
- Integrate visual meshes with trained walking policy
