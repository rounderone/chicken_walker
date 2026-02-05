# Blender script to import and assemble AT-ST from Thingiverse STL files
# Run this inside Blender: File -> Scripting -> Open -> Run Script
# Or from command line: blender --python assemble_atst.py

import bpy
import os
import math

# Clear existing objects
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# Base path to STL files
THINGIVERSE_BASE = "D:/Projects/ATST/thingiverse"
PART1 = f"{THINGIVERSE_BASE}/AT-ST Highly Detailed and Posable - 6986726 - part 1 of 3/files"
PART2 = f"{THINGIVERSE_BASE}/AT-ST Highly Detailed and Posable - 6986726 - part 2 of 3/files"

# Scale: STL files are in mm, Blender default is meters
# 1/48 scale model is ~216mm tall, we'll keep it in mm for now and scale later
MM_TO_M = 0.001

def import_stl(filepath, name=None):
    """Import an STL file and return the object."""
    if not os.path.exists(filepath):
        print(f"WARNING: File not found: {filepath}")
        return None

    bpy.ops.wm.stl_import(filepath=filepath)
    obj = bpy.context.active_object

    if name:
        obj.name = name

    # Convert mm to meters
    obj.scale = (MM_TO_M, MM_TO_M, MM_TO_M)

    return obj

def create_collection(name):
    """Create a collection for organizing parts."""
    collection = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(collection)
    return collection

def move_to_collection(obj, collection):
    """Move an object to a specific collection."""
    # Remove from all current collections
    for col in obj.users_collection:
        col.objects.unlink(obj)
    # Add to new collection
    collection.objects.link(obj)

print("="*60)
print("IMPORTING AT-ST PARTS")
print("="*60)

# Create collections for organization
head_col = create_collection("Head_Cockpit")
body_col = create_collection("Body")
left_leg_col = create_collection("Left_Leg")
right_leg_col = create_collection("Right_Leg")
weapons_col = create_collection("Weapons")

# =============================================================================
# HEAD/COCKPIT PARTS
# =============================================================================
print("\nImporting head/cockpit parts...")

head_parts = [
    (f"{PART1}/AT-ST_Head_Main_a_x1.stl", "Head_Main_A"),
    (f"{PART1}/AT-ST_Head_Main_b_x1.stl", "Head_Main_B"),
    (f"{PART1}/AT-ST_Head_Face__x1.stl", "Head_Face"),
    (f"{PART1}/AT-ST_Head_Power__x1.stl", "Head_Power"),
    (f"{PART1}/AT-ST_Head_Cables_x1.stl", "Head_Cables"),
    (f"{PART1}/AT-ST_Head_Neck_x1.stl", "Head_Neck"),
    (f"{PART1}/AT-ST_Head_Armor_Lg_right_x1.stl", "Head_Armor_Lg_R"),
    (f"{PART1}/AT-ST_Head_Armor_Lg_left_x1.stl", "Head_Armor_Lg_L"),
    (f"{PART1}/AT-ST_Head_Armor_Sm_A_right_x1.stl", "Head_Armor_Sm_A_R"),
    (f"{PART1}/AT-ST_Head_Armor_Sm_A_left_x1.stl", "Head_Armor_Sm_A_L"),
    (f"{PART1}/AT-ST_Head_AC_x1.stl", "Head_AC"),
]

for filepath, name in head_parts:
    obj = import_stl(filepath, name)
    if obj:
        move_to_collection(obj, head_col)
        print(f"  Imported: {name}")

# =============================================================================
# BODY PARTS
# =============================================================================
print("\nImporting body parts...")

body_parts = [
    (f"{PART1}/AT-ST_Body_x1.stl", "Body_Main"),
    (f"{PART1}/AT-ST_Body_Power__x1.stl", "Body_Power"),
    (f"{PART1}/AT-ST_Body_Exhaust_x1.stl", "Body_Exhaust"),
    (f"{PART1}/AT-ST_Body_Thingy__x1.stl", "Body_Connector"),
    (f"{PART1}/AT-ST_Body_Armor_upper_right__x1.stl", "Body_Armor_Upper_R"),
    (f"{PART1}/AT-ST_Body_Armor_upper_left__x1.stl", "Body_Armor_Upper_L"),
    (f"{PART2}/AT-ST_Body_Armor_lower_right__x1.stl", "Body_Armor_Lower_R"),
    (f"{PART2}/AT-ST_Body_Armor_lower_left__x1.stl", "Body_Armor_Lower_L"),
]

for filepath, name in body_parts:
    obj = import_stl(filepath, name)
    if obj:
        move_to_collection(obj, body_col)
        print(f"  Imported: {name}")

# =============================================================================
# LEFT LEG PARTS
# =============================================================================
print("\nImporting left leg parts...")

left_leg_parts = [
    (f"{PART2}/AT-ST_Legs_Hip_x2.stl", "L_Hip"),
    (f"{PART2}/AT-ST_Legs_Upper_left_a_x1.stl", "L_Upper_A"),
    (f"{PART2}/AT-ST_Legs_Upper_left_b_x1.stl", "L_Upper_B"),
    (f"{PART2}/AT-ST_Legs_Upper_Armor_left_x1.stl", "L_Upper_Armor"),
    (f"{PART2}/AT-ST_Legs_Upper_Kneecap_x2.stl", "L_Kneecap"),
    (f"{PART2}/AT-ST_Legs_Mid_left_a_x1.stl", "L_Mid_A"),
    (f"{PART2}/AT-ST_Legs_Mid_left_b_x1.stl", "L_Mid_B"),
    (f"{PART2}/AT-ST_Legs_Mid_LeafSpring_x2.stl", "L_LeafSpring"),
    (f"{PART2}/AT-ST_Legs_Lower_left_a_x1.stl", "L_Lower_A"),
    (f"{PART2}/AT-ST_Legs_Lower_left_b_x1.stl", "L_Lower_B"),
    (f"{PART2}/AT-ST_Foot_left__x1.stl", "L_Foot"),
    (f"{PART2}/AT-ST_Foot_Toe_left__x1.stl", "L_Toe"),
]

for filepath, name in left_leg_parts:
    obj = import_stl(filepath, name)
    if obj:
        move_to_collection(obj, left_leg_col)
        print(f"  Imported: {name}")

# =============================================================================
# RIGHT LEG PARTS
# =============================================================================
print("\nImporting right leg parts...")

right_leg_parts = [
    (f"{PART2}/AT-ST_Legs_Upper_right_a_x1.stl", "R_Upper_A"),
    (f"{PART2}/AT-ST_Legs_Upper_right_b_x1.stl", "R_Upper_B"),
    (f"{PART2}/AT-ST_Legs_Upper_Armor_right_x1.stl", "R_Upper_Armor"),
    (f"{PART2}/AT-ST_Legs_Mid_right_a_x1.stl", "R_Mid_A"),
    (f"{PART2}/AT-ST_Legs_Mid_right_b_x1.stl", "R_Mid_B"),
    (f"{PART2}/AT-ST_Legs_Lower_right_a_x1.stl", "R_Lower_A"),
    (f"{PART2}/AT-ST_Legs_Lower_right_b_x1.stl", "R_Lower_B"),
    (f"{PART2}/AT-ST_Foot_right__x1.stl", "R_Foot"),
    (f"{PART2}/AT-ST_Foot_Toe_right__x1.stl", "R_Toe"),
]

for filepath, name in right_leg_parts:
    obj = import_stl(filepath, name)
    if obj:
        move_to_collection(obj, right_leg_col)
        print(f"  Imported: {name}")

# =============================================================================
# WEAPONS
# =============================================================================
print("\nImporting weapons...")

weapon_parts = [
    (f"{PART1}/AT-ST_Guns_front_x1.stl", "Gun_Front"),
    (f"{PART1}/AT-ST_Guns_right__x1.stl", "Gun_Right"),
    (f"{PART1}/AT-ST_Guns_left__x1.stl", "Gun_Left"),
]

for filepath, name in weapon_parts:
    obj = import_stl(filepath, name)
    if obj:
        move_to_collection(obj, weapons_col)
        print(f"  Imported: {name}")

# =============================================================================
# APPLY TRANSFORMS AND SETUP VIEW
# =============================================================================
print("\nApplying transforms...")

# Apply scale to all objects
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

# Frame all objects in view
bpy.ops.object.select_all(action='SELECT')
for area in bpy.context.screen.areas:
    if area.type == 'VIEW_3D':
        for region in area.regions:
            if region.type == 'WINDOW':
                override = {'area': area, 'region': region}
                bpy.ops.view3d.view_selected(override)
                break

print("\n" + "="*60)
print("IMPORT COMPLETE!")
print("="*60)
print("\nParts are organized in collections:")
print("  - Head_Cockpit")
print("  - Body")
print("  - Left_Leg")
print("  - Right_Leg")
print("  - Weapons")
print("\nNow manually position the parts to assemble the AT-ST.")
print("Tips:")
print("  - Use G to grab/move, R to rotate, S to scale")
print("  - Press N to open the sidebar for precise positioning")
print("  - Hide collections by clicking the eye icon in Outliner")
print("\nOnce assembled, export as USD:")
print("  File -> Export -> Universal Scene Description (.usd)")
print("  Save to: D:/Projects/ATST/usd/atst_assembled.usda")
