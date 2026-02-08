# AT-ST Project - TODO

## Current State: Deep Crouch Working!

The AT-ST can now perform a deep crouch (60 degree knee bend) while keeping the torso level using hip compensation.

---

## Immediate Next Steps

### Fine-tune Deeper Crouch
- [ ] Test K70 and K80 knee angles with adjusted hip ratios
- [ ] Current working: K60 with hip_ratio=0.3 (gives +18 degree hip)
- [ ] Try hip_ratio=0.35-0.4 for deeper crouch stability

### Add Walking Gait
- [ ] Implement alternating leg lift pattern
- [ ] Maintain hip compensation during weight shifts
- [ ] Test shift-from-foot-to-foot before full walking

---

## Completed (Golden Path)

### Phase 1: 12-Part Articulated Model
- [x] Separated bar geometry from thigh in Blender
- [x] Exported atst_with_bars.usda with 12 parts
- [x] User verified joint positions

### Phase 2: Physics Setup
- [x] ArticulationRootAPI on torso (floating base)
- [x] RevoluteJoints with position drive (stiffness=5000, damping=500)
- [x] Convex hull collision on feet, shins, elbows
- [x] No collision on thighs/bars (prevent self-intersection)

### Phase 3: Deep Crouch
- [x] Found working configuration: K60 H+18
- [x] Hip compensation ratio: 0.3
- [x] Robot stays level during full crouch cycle

---

## Key Files (Use These!)

| File | Purpose |
|------|---------|
| `usd/atst_with_bars.usda` | The working model (12 parts) |
| `usd/202602072308_joints.usda` | User-verified joint positions |
| `isaac_sim/test_nine_behaviors.py` | Working test script |
| `checkpoints/` | Milestone backups (2026-02-08) |

---

## Quick Start for New Conversation

```
I'm continuing the AT-ST project. The robot can now crouch deeply (K60 knee)
with hip compensation (ratio 0.3). Files are in checkpoints/ folder.
Next: fine-tune deeper crouch or add walking gait.
```

---

## Working Configuration Reference

```python
# Crouch parameters that work
knee_max = 60
hip_ratio = 0.3  # hip_angle = knee_max * hip_ratio * crouch_amount

# Part masses
PART_MASSES = {
    'ATST_torso': 20.0, 'ATST_head': 10.0,
    'ATST_bar_L': 2.0, 'ATST_bar_R': 2.0,
    'ATST_thigh_L': 4.0, 'ATST_thigh_R': 4.0,
    'ATST_shin_L': 4.0, 'ATST_shin_R': 4.0,
    'ATST_elbow_L': 3.0, 'ATST_elbow_R': 3.0,
    'ATST_foot_L': 15.0, 'ATST_foot_R': 15.0,
}

# Disable collision on these parts
NO_COLLISION_PARTS = {'ATST_thigh_L', 'ATST_thigh_R', 'ATST_bar_L', 'ATST_bar_R'}
```
