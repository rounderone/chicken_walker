# Run this in Blender to see what vertex groups exist
# Scripting tab -> New -> Paste this -> Run

import bpy

print("="*60)
print("CHECKING VERTEX GROUPS IN MESHES")
print("="*60)

for obj in bpy.data.objects:
    if obj.type == 'MESH' and obj.vertex_groups:
        print(f"\n{obj.name}:")
        print(f"  Vertices: {len(obj.data.vertices)}")
        print(f"  Vertex Groups ({len(obj.vertex_groups)}):")
        for vg in obj.vertex_groups:
            # Count vertices in this group
            count = 0
            for v in obj.data.vertices:
                for g in v.groups:
                    if g.group == vg.index and g.weight > 0.1:
                        count += 1
                        break
            print(f"    {vg.name}: {count} verts")
