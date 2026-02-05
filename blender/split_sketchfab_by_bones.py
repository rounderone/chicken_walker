# Blender script to split Sketchfab AT-ST mesh by bone weights
# This takes the monolithic mesh and separates it into parts based on armature bones
#
# Run in Blender: Scripting tab -> Open -> Run Script
#
# First, import the Sketchfab .glb or .fbx file into Blender

import bpy
import os

print("="*60)
print("SPLITTING AT-ST MESH BY BONE WEIGHTS")
print("="*60)

# First, check what's in the scene
print("\nObjects in scene:")
for obj in bpy.data.objects:
    print(f"  {obj.name} ({obj.type})")

# Find the armature and mesh
armature = None
mesh_obj = None

for obj in bpy.data.objects:
    if obj.type == 'ARMATURE':
        armature = obj
        print(f"\nFound armature: {obj.name}")
    elif obj.type == 'MESH':
        # Look for the main AT-ST mesh (has the most vertices)
        if mesh_obj is None or len(obj.data.vertices) > len(mesh_obj.data.vertices):
            mesh_obj = obj

if mesh_obj:
    print(f"Found mesh: {mesh_obj.name} ({len(mesh_obj.data.vertices)} vertices)")

if not armature or not mesh_obj:
    print("\nERROR: Could not find armature or mesh!")
    print("Please import the Sketchfab AT-ST model first:")
    print("  File -> Import -> glTF 2.0 (.glb/.gltf)")
    print("  Or download the .blend file from Sketchfab")
else:
    # List all bones in the armature
    print(f"\nBones in armature ({len(armature.data.bones)}):")
    for bone in armature.data.bones:
        print(f"  {bone.name}")

    # List vertex groups in the mesh (should match bones)
    print(f"\nVertex groups in mesh ({len(mesh_obj.vertex_groups)}):")
    for vg in mesh_obj.vertex_groups:
        print(f"  {vg.name}")

    # Define which bones/vertex groups to use for splitting
    # We want the main body parts, not every tiny bone
    BODY_PARTS = {
        'head': ['head', 'Front_turret', 'Side_Turret_L', 'Side_Turret_R'],
        'body': ['ATST', 'Body'],
        'hip_L': ['Torso_L', 'Hip1_L', 'Hip2_L', 'Hip3_L'],
        'thigh_L': ['Thigh_L'],
        'shin_L': ['Shin_L'],
        'foot_L': ['Ankle_L', 'Foot_L'],
        'hip_R': ['Torso_R', 'Hip1_R', 'Hip2_R', 'Hip3_R'],
        'thigh_R': ['Thigh_R'],
        'shin_R': ['Shin_R'],
        'foot_R': ['Ankle_R', 'Foot_R'],
    }

    print("\n" + "="*60)
    print("SPLITTING MESH BY BODY PARTS")
    print("="*60)

    # Make sure we're in object mode
    bpy.ops.object.mode_set(mode='OBJECT')

    # Deselect all
    bpy.ops.object.select_all(action='DESELECT')

    # Select and make the mesh active
    mesh_obj.select_set(True)
    bpy.context.view_layer.objects.active = mesh_obj

    # For each body part, select vertices and separate
    created_parts = []

    for part_name, bone_names in BODY_PARTS.items():
        print(f"\nProcessing: {part_name}")

        # Go to edit mode
        bpy.ops.object.mode_set(mode='EDIT')

        # Deselect all vertices
        bpy.ops.mesh.select_all(action='DESELECT')

        # Go to object mode to access vertex groups properly
        bpy.ops.object.mode_set(mode='OBJECT')

        # Select vertices from each vertex group for this part
        for bone_name in bone_names:
            if bone_name in mesh_obj.vertex_groups:
                vg = mesh_obj.vertex_groups[bone_name]
                vg_index = vg.index

                # Select vertices in this group
                for v in mesh_obj.data.vertices:
                    for g in v.groups:
                        if g.group == vg_index and g.weight > 0.5:  # Only well-weighted verts
                            v.select = True
                            break

                print(f"  Added vertices from: {bone_name}")
            else:
                print(f"  WARNING: Vertex group '{bone_name}' not found")

        # Go back to edit mode
        bpy.ops.object.mode_set(mode='EDIT')

        # Check if we have any selected vertices
        bpy.ops.object.mode_set(mode='OBJECT')
        selected_count = sum(1 for v in mesh_obj.data.vertices if v.select)

        if selected_count > 0:
            print(f"  Selected {selected_count} vertices")

            # Separate selected vertices
            bpy.ops.object.mode_set(mode='EDIT')
            try:
                bpy.ops.mesh.separate(type='SELECTED')
                print(f"  Separated into new object")
                created_parts.append(part_name)
            except:
                print(f"  Could not separate (might already be separate)")

            bpy.ops.object.mode_set(mode='OBJECT')

            # Rename the newly created object
            for obj in bpy.context.selected_objects:
                if obj != mesh_obj and obj.name.startswith(mesh_obj.name):
                    obj.name = f"ATST_{part_name}"
                    print(f"  Renamed to: {obj.name}")
        else:
            print(f"  No vertices selected for {part_name}")

        # Reselect the original mesh for the next iteration
        bpy.ops.object.select_all(action='DESELECT')
        mesh_obj.select_set(True)
        bpy.context.view_layer.objects.active = mesh_obj

    print("\n" + "="*60)
    print("SPLIT COMPLETE")
    print("="*60)
    print(f"Created {len(created_parts)} body parts:")
    for part in created_parts:
        print(f"  ATST_{part}")

    print("\nNext steps:")
    print("1. Review the split parts in the 3D viewport")
    print("2. Clean up any remaining original mesh")
    print("3. Export as USD: File -> Export -> Universal Scene Description")
    print("   Save to: D:/Projects/ATST/usd/atst_split.usda")
