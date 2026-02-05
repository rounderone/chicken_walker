# Re-analyze Sketchfab AT-ST model for armature/skeleton structure
# Run with: D:\Projects\ATST\venv_isaaclab\Scripts\python.exe reanalyze_sketchfab.py

print("Analyzing Sketchfab AT-ST model...", flush=True)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdSkel
import os

model_path = 'D:/Projects/ATST/usd/atst.usdc'
if not os.path.exists(model_path):
    print(f'Model not found at {model_path}')
    simulation_app.close()
    exit()

stage = Usd.Stage.Open(model_path)
print(f"\nOpened: {model_path}", flush=True)
print()

# Find all prims and their types
meshes = []
skeletons = []
xforms = []
joints = []
skel_roots = []
all_prims = []

for prim in stage.Traverse():
    ptype = prim.GetTypeName()
    path = str(prim.GetPath())
    all_prims.append((path, ptype))

    if prim.IsA(UsdGeom.Mesh):
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        num_verts = len(points) if points else 0
        meshes.append((path, num_verts))
    elif ptype == "SkelRoot":
        skel_roots.append(path)
    elif ptype == "Skeleton":
        skeletons.append(path)
        # Try to get joint names
        skel = UsdSkel.Skeleton(prim)
        joints_attr = skel.GetJointsAttr()
        if joints_attr:
            joint_names = joints_attr.Get()
            if joint_names:
                for j in joint_names:
                    joints.append(j)
    elif prim.IsA(UsdGeom.Xform):
        xforms.append(path)

print(f'='*60, flush=True)
print('MESH ANALYSIS', flush=True)
print(f'='*60, flush=True)
print(f'Meshes found: {len(meshes)}', flush=True)
for path, verts in meshes:
    name = path.split('/')[-1]
    print(f'  {name}: {verts} vertices', flush=True)

print(f'\n' + '='*60, flush=True)
print('SKELETON/ARMATURE ANALYSIS', flush=True)
print(f'='*60, flush=True)
print(f'SkelRoot prims: {len(skel_roots)}', flush=True)
for s in skel_roots:
    print(f'  {s}', flush=True)

print(f'\nSkeleton prims: {len(skeletons)}', flush=True)
for s in skeletons:
    print(f'  {s}', flush=True)

print(f'\nJoints/Bones in skeleton: {len(joints)}', flush=True)
for j in joints:
    print(f'  {j}', flush=True)

print(f'\n' + '='*60, flush=True)
print('HIERARCHY (depth <= 3)', flush=True)
print(f'='*60, flush=True)
for path, ptype in all_prims:
    depth = path.count('/')
    if depth <= 3:
        indent = '  ' * (depth - 1)
        name = path.split('/')[-1]
        print(f'{indent}{name} ({ptype})', flush=True)

simulation_app.close()
print("\nDone!", flush=True)
