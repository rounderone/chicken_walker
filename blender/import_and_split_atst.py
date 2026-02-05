# Blender script to import AT-ST FBX and split by bone weights
# Run with: blender --python import_and_split_atst.py

import bpy
import os

print("="*60)
print("AT-ST IMPORT AND SPLIT SCRIPT")
print("="*60)

# Clear existing objects
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# Import the FBX file
fbx_path = "D:/Projects/ATST/models/source/ATST workshop.fbx"
print(f"\nImporting: {fbx_path}")

if os.path.exists(fbx_path):
    bpy.ops.import_scene.fbx(filepath=fbx_path)
    print("Import successful!")
else:
    print(f"ERROR: File not found: {fbx_path}")
    # Don't exit - let user see the error

# List what was imported
print("\nObjects imported:")
armature = None
mesh_objects = []

for obj in bpy.data.objects:
    print(f"  {obj.name} ({obj.type})")
    if obj.type == 'ARMATURE':
        armature = obj
    elif obj.type == 'MESH':
        mesh_objects.append(obj)

if armature:
    print(f"\nArmature found: {armature.name}")
    print(f"Bones: {len(armature.data.bones)}")
    for bone in armature.data.bones:
        # Only show main bones, not end bones
        if '_end' not in bone.name.lower():
            print(f"  {bone.name}")

if mesh_objects:
    print(f"\nMesh objects: {len(mesh_objects)}")
    for mesh in mesh_objects:
        verts = len(mesh.data.vertices) if mesh.data else 0
        print(f"  {mesh.name}: {verts} vertices")
        if mesh.vertex_groups:
            print(f"    Vertex groups: {len(mesh.vertex_groups)}")

# Find the main mesh (largest one)
main_mesh = None
if mesh_objects:
    main_mesh = max(mesh_objects, key=lambda m: len(m.data.vertices) if m.data else 0)
    print(f"\nMain mesh identified: {main_mesh.name}")

# Define body parts to split
# Mapping bone names to body part names
BODY_PARTS = {
    'ATST_head': ['head'],
    'ATST_body': ['Body', 'ATST'],  # Root bones
    'ATST_hip_L': ['Torso_L', 'Hip1_L', 'Hip2_L', 'Hip3_L'],
    'ATST_thigh_L': ['Thigh_L'],
    'ATST_shin_L': ['Shin_L'],
    'ATST_ankle_L': ['Ankle_L'],
    'ATST_foot_L': ['Foot_L'],
    'ATST_hip_R': ['Torso_R', 'Hip1_R', 'Hip2_R', 'Hip3_R'],
    'ATST_thigh_R': ['Thigh_R'],
    'ATST_shin_R': ['Shin_R'],
    'ATST_ankle_R': ['Ankle_R'],
    'ATST_foot_R': ['Foot_R'],
}

if main_mesh and main_mesh.vertex_groups:
    print("\n" + "="*60)
    print("SPLITTING MESH BY VERTEX GROUPS")
    print("="*60)

    # List available vertex groups
    vg_names = [vg.name for vg in main_mesh.vertex_groups]
    print(f"\nAvailable vertex groups: {vg_names}")

    # Make the main mesh active
    bpy.ops.object.select_all(action='DESELECT')
    main_mesh.select_set(True)
    bpy.context.view_layer.objects.active = main_mesh

    created_parts = []

    for part_name, bone_names in BODY_PARTS.items():
        print(f"\nProcessing: {part_name}")

        # Find matching vertex groups
        matching_vgs = []
        for bone_name in bone_names:
            for vg_name in vg_names:
                if bone_name.lower() in vg_name.lower():
                    matching_vgs.append(vg_name)

        if not matching_vgs:
            print(f"  No matching vertex groups found for {bone_names}")
            continue

        print(f"  Matching vertex groups: {matching_vgs}")

        # Select vertices from matching groups
        bpy.ops.object.mode_set(mode='OBJECT')

        # Deselect all vertices
        for v in main_mesh.data.vertices:
            v.select = False

        # Select vertices in the matching groups
        for vg_name in matching_vgs:
            if vg_name in main_mesh.vertex_groups:
                vg = main_mesh.vertex_groups[vg_name]
                vg_index = vg.index

                for v in main_mesh.data.vertices:
                    for g in v.groups:
                        if g.group == vg_index and g.weight > 0.1:
                            v.select = True

        # Count selected
        selected_count = sum(1 for v in main_mesh.data.vertices if v.select)
        print(f"  Selected {selected_count} vertices")

        if selected_count > 100:  # Only separate if we have significant geometry
            # Separate selected vertices
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.separate(type='SELECTED')
            bpy.ops.object.mode_set(mode='OBJECT')

            # Find and rename the new object
            for obj in bpy.context.selected_objects:
                if obj != main_mesh and obj.type == 'MESH':
                    obj.name = part_name
                    created_parts.append(part_name)
                    print(f"  Created: {part_name}")
                    break

            # Reselect main mesh
            bpy.ops.object.select_all(action='DESELECT')
            main_mesh.select_set(True)
            bpy.context.view_layer.objects.active = main_mesh

    print("\n" + "="*60)
    print("SPLIT COMPLETE")
    print("="*60)
    print(f"\nCreated {len(created_parts)} body parts:")
    for part in created_parts:
        print(f"  {part}")

    # Rename remaining mesh as body
    if main_mesh:
        main_mesh.name = "ATST_body_remaining"
        print(f"\nRemaining mesh renamed to: {main_mesh.name}")

else:
    print("\nNo vertex groups found - mesh may not be rigged.")
    print("The mesh cannot be auto-split without vertex groups.")

# Frame all in view
bpy.ops.object.select_all(action='SELECT')
for area in bpy.context.screen.areas:
    if area.type == 'VIEW_3D':
        for region in area.regions:
            if region.type == 'WINDOW':
                with bpy.context.temp_override(area=area, region=region):
                    bpy.ops.view3d.view_selected()
                break

# Save the file
output_path = "D:/Projects/ATST/blender/atst_split.blend"
bpy.ops.wm.save_as_mainfile(filepath=output_path)
print(f"\nSaved to: {output_path}")

print("\n" + "="*60)
print("DONE!")
print("="*60)
print("\nNext: Export as USD for Isaac Sim")
print("  File -> Export -> Universal Scene Description (.usd)")
print("  Save to: D:/Projects/ATST/usd/atst_split.usda")
