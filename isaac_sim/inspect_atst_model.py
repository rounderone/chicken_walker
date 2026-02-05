# Inspect the Sketchfab AT-ST model to see what meshes it contains
# Run with: D:\Projects\ATST\venv_isaaclab\Scripts\python.exe inspect_atst_model.py

print("Inspecting Sketchfab AT-ST model...", flush=True)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdShade, Gf
import omni.usd

# Open the Sketchfab model
model_path = "D:/Projects/ATST/usd/atst.usdc"
print(f"\nOpening: {model_path}", flush=True)

stage = Usd.Stage.Open(model_path)

if not stage:
    print("ERROR: Could not open stage!")
    simulation_app.close()
    exit(1)

print(f"Stage opened successfully!", flush=True)
print(f"Default prim: {stage.GetDefaultPrim()}", flush=True)

# Count different prim types
meshes = []
xforms = []
materials = []
other = []

print("\n" + "="*60, flush=True)
print("SCANNING ALL PRIMS", flush=True)
print("="*60, flush=True)

for prim in stage.Traverse():
    ptype = prim.GetTypeName()
    path = str(prim.GetPath())

    if prim.IsA(UsdGeom.Mesh):
        meshes.append(prim)
    elif prim.IsA(UsdGeom.Xform):
        xforms.append(prim)
    elif prim.IsA(UsdShade.Material):
        materials.append(prim)
    else:
        other.append((path, ptype))

print(f"\nSummary:", flush=True)
print(f"  Meshes: {len(meshes)}", flush=True)
print(f"  Xforms: {len(xforms)}", flush=True)
print(f"  Materials: {len(materials)}", flush=True)
print(f"  Other: {len(other)}", flush=True)

# Print mesh details
print("\n" + "="*60, flush=True)
print("MESH DETAILS", flush=True)
print("="*60, flush=True)

for i, mesh_prim in enumerate(meshes):
    mesh = UsdGeom.Mesh(mesh_prim)
    path = str(mesh_prim.GetPath())

    # Get mesh name from path
    name = path.split("/")[-1]

    # Get vertex count
    points = mesh.GetPointsAttr().Get()
    num_verts = len(points) if points else 0

    # Get face count
    face_counts = mesh.GetFaceVertexCountsAttr().Get()
    num_faces = len(face_counts) if face_counts else 0

    # Get bounding box
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bbox = bbox_cache.ComputeWorldBound(mesh_prim)
    bbox_range = bbox.GetRange()

    if not bbox_range.IsEmpty():
        min_pt = bbox_range.GetMin()
        max_pt = bbox_range.GetMax()
        size = bbox_range.GetSize()
        center = (min_pt + max_pt) / 2

        print(f"\n{i+1}. {name}", flush=True)
        print(f"   Path: {path}", flush=True)
        print(f"   Vertices: {num_verts}, Faces: {num_faces}", flush=True)
        print(f"   Bounding box size: ({size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f})", flush=True)
        print(f"   Center: ({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f})", flush=True)
    else:
        print(f"\n{i+1}. {name}", flush=True)
        print(f"   Path: {path}", flush=True)
        print(f"   Vertices: {num_verts}, Faces: {num_faces}", flush=True)
        print(f"   Bounding box: empty/invalid", flush=True)

# Print xform hierarchy (top level only)
print("\n" + "="*60, flush=True)
print("XFORM HIERARCHY (Top 3 levels)", flush=True)
print("="*60, flush=True)

for xform_prim in xforms:
    path = str(xform_prim.GetPath())
    depth = path.count("/")
    if depth <= 3:
        indent = "  " * (depth - 1)
        name = path.split("/")[-1]
        print(f"{indent}{name}", flush=True)

# Check for skeleton/armature
print("\n" + "="*60, flush=True)
print("CHECKING FOR RIGGING", flush=True)
print("="*60, flush=True)

has_skeleton = False
has_joints = False

for prim in stage.Traverse():
    ptype = prim.GetTypeName()
    path = str(prim.GetPath())

    if "skeleton" in ptype.lower() or "skeleton" in path.lower():
        print(f"Found skeleton: {path} ({ptype})", flush=True)
        has_skeleton = True

    if "joint" in path.lower() or ptype == "Joint":
        print(f"Found joint: {path} ({ptype})", flush=True)
        has_joints = True

    if "armature" in path.lower():
        print(f"Found armature: {path} ({ptype})", flush=True)

if not has_skeleton and not has_joints:
    print("No skeleton or joints found - model is likely static meshes only", flush=True)

# Check mesh names for body part hints
print("\n" + "="*60, flush=True)
print("BODY PART DETECTION (by mesh name)", flush=True)
print("="*60, flush=True)

body_keywords = ["body", "cockpit", "head", "torso", "cabin", "hull"]
leg_keywords = ["leg", "thigh", "shin", "calf", "femur", "tibia"]
foot_keywords = ["foot", "feet", "toe", "pad"]

for mesh_prim in meshes:
    name = str(mesh_prim.GetPath()).lower()

    for kw in body_keywords:
        if kw in name:
            print(f"Body part: {mesh_prim.GetPath()}", flush=True)
            break

    for kw in leg_keywords:
        if kw in name:
            print(f"Leg part: {mesh_prim.GetPath()}", flush=True)
            break

    for kw in foot_keywords:
        if kw in name:
            print(f"Foot part: {mesh_prim.GetPath()}", flush=True)
            break

print("\n" + "="*60, flush=True)
print("INSPECTION COMPLETE", flush=True)
print("="*60, flush=True)

simulation_app.close()
print("Done!", flush=True)
