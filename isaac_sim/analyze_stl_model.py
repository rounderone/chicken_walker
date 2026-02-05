# Analyze Thingiverse STL files to understand scale and structure
# Run with: D:\Projects\ATST\venv_isaaclab\Scripts\python.exe analyze_stl_model.py
#
# This script imports all STL files and analyzes their dimensions to understand:
# 1. Total model scale (likely in mm, need to convert to meters)
# 2. Where body parts are positioned relative to each other
# 3. What dimensions our physics skeleton should have

import os
import sys
print("Analyzing Thingiverse AT-ST STL files...", flush=True)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

print("Isaac Sim started!", flush=True)

from pxr import UsdGeom, Usd, Gf
import omni.usd
import omni.kit.commands

stage = omni.usd.get_context().get_stage()

# =============================================================================
# STL file paths
# =============================================================================
THINGIVERSE_BASE = "D:/Projects/ATST/thingiverse"
PART1 = f"{THINGIVERSE_BASE}/AT-ST Highly Detailed and Posable - 6986726 - part 1 of 3/files"
PART2 = f"{THINGIVERSE_BASE}/AT-ST Highly Detailed and Posable - 6986726 - part 2 of 3/files"


def import_stl(stl_path, parent_path):
    """Import an STL file and return its bounding box."""
    if not os.path.exists(stl_path):
        print(f"  WARNING: File not found: {stl_path}", flush=True)
        return None

    filename = os.path.basename(stl_path).replace(".stl", "").replace(" ", "_")
    prim_path = f"{parent_path}/{filename}"

    # Import STL using asset converter
    try:
        import asyncio
        from omni.kit.asset_converter import AssetConverterContext
        from omni.kit.asset_converter.impl import AssetConverterImpl

        converter = AssetConverterImpl()
        context = AssetConverterContext()

        # Create a simple mesh prim instead since direct STL import is complex
        # We'll use the reference approach

        # Actually, let's just create a reference to the STL
        # Isaac Sim can import STL files via the asset converter

        success = omni.kit.commands.execute(
            "CreateMeshPrimCommand",
            prim_type="Mesh",
            prim_path=prim_path
        )

        if not success:
            return None

    except Exception as e:
        print(f"  Error importing {filename}: {e}", flush=True)
        return None

    return prim_path


def get_stl_bounds_via_numpy(stl_path):
    """Read STL file directly to get bounding box using numpy-stl."""
    try:
        from stl import mesh
        stl_mesh = mesh.Mesh.from_file(stl_path)

        # Get min/max across all vertices
        min_pt = stl_mesh.vectors.min(axis=(0, 1))
        max_pt = stl_mesh.vectors.max(axis=(0, 1))

        return min_pt, max_pt
    except ImportError:
        return None, None
    except Exception as e:
        print(f"  Error reading {os.path.basename(stl_path)}: {e}", flush=True)
        return None, None


# Try to install numpy-stl if not available
try:
    from stl import mesh
    print("numpy-stl available for STL parsing", flush=True)
except ImportError:
    print("Installing numpy-stl for STL parsing...", flush=True)
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "numpy-stl", "-q"])
    from stl import mesh
    print("numpy-stl installed!", flush=True)

# =============================================================================
# Analyze key body parts
# =============================================================================
print("\n" + "="*60, flush=True)
print("ANALYZING STL DIMENSIONS", flush=True)
print("(Assuming STL files are in millimeters)", flush=True)
print("="*60, flush=True)

# Key files to analyze
key_files = {
    "Head/Body": f"{PART1}/AT-ST_Body_x1.stl",
    "Head Main A": f"{PART1}/AT-ST_Head_Main_a_x1.stl",
    "Head Main B": f"{PART1}/AT-ST_Head_Main_b_x1.stl",
    "Neck": f"{PART1}/AT-ST_Head_Neck_x1.stl",
    "Hip": f"{PART2}/AT-ST_Legs_Hip_x2.stl",
    "Upper Leg A (left)": f"{PART2}/AT-ST_Legs_Upper_left_a_x1.stl",
    "Upper Leg B (left)": f"{PART2}/AT-ST_Legs_Upper_left_b_x1.stl",
    "Mid Leg A (left)": f"{PART2}/AT-ST_Legs_Mid_left_a_x1.stl",
    "Mid Leg B (left)": f"{PART2}/AT-ST_Legs_Mid_left_b_x1.stl",
    "Lower Leg A (left)": f"{PART2}/AT-ST_Legs_Lower_left_a_x1.stl",
    "Lower Leg B (left)": f"{PART2}/AT-ST_Legs_Lower_left_b_x1.stl",
    "Foot (left)": f"{PART2}/AT-ST_Foot_left__x1.stl",
    "Foot Toe (left)": f"{PART2}/AT-ST_Foot_Toe_left__x1.stl",
}

all_min = [float('inf'), float('inf'), float('inf')]
all_max = [float('-inf'), float('-inf'), float('-inf')]

for name, filepath in key_files.items():
    min_pt, max_pt = get_stl_bounds_via_numpy(filepath)
    if min_pt is not None:
        size = max_pt - min_pt
        print(f"\n{name}:", flush=True)
        print(f"  File: {os.path.basename(filepath)}", flush=True)
        print(f"  Min (mm): ({min_pt[0]:.1f}, {min_pt[1]:.1f}, {min_pt[2]:.1f})", flush=True)
        print(f"  Max (mm): ({max_pt[0]:.1f}, {max_pt[1]:.1f}, {max_pt[2]:.1f})", flush=True)
        print(f"  Size (mm): ({size[0]:.1f}, {size[1]:.1f}, {size[2]:.1f})", flush=True)
        print(f"  Size (m):  ({size[0]/1000:.3f}, {size[1]/1000:.3f}, {size[2]/1000:.3f})", flush=True)

        # Track overall bounds
        for i in range(3):
            all_min[i] = min(all_min[i], min_pt[i])
            all_max[i] = max(all_max[i], max_pt[i])

# Print overall analysis
print("\n" + "="*60, flush=True)
print("OVERALL MODEL BOUNDS (from key parts)", flush=True)
print("="*60, flush=True)
total_size = [all_max[i] - all_min[i] for i in range(3)]
print(f"Min (mm): ({all_min[0]:.1f}, {all_min[1]:.1f}, {all_min[2]:.1f})", flush=True)
print(f"Max (mm): ({all_max[0]:.1f}, {all_max[1]:.1f}, {all_max[2]:.1f})", flush=True)
print(f"Total Size (mm): ({total_size[0]:.1f}, {total_size[1]:.1f}, {total_size[2]:.1f})", flush=True)
print(f"Total Size (m):  ({total_size[0]/1000:.3f}, {total_size[1]/1000:.3f}, {total_size[2]/1000:.3f})", flush=True)

# =============================================================================
# Determine scale factor
# =============================================================================
print("\n" + "="*60, flush=True)
print("SCALE ANALYSIS", flush=True)
print("="*60, flush=True)

# AT-ST in Star Wars is about 8.6 meters tall
# But for RL we want a smaller ~1m model for faster training
# Let's see what scale factor gives us a reasonable robot

# If the STL is in mm and we want ~1m tall robot:
# height_mm / scale_factor = target_height_m * 1000
# scale_factor = height_mm / (target_height_m * 1000)

estimated_height_mm = total_size[2]  # Z is typically up
target_height_m = 0.8  # Target ~0.8m for our walker

scale_factor = 1000 / estimated_height_mm * target_height_m if estimated_height_mm > 0 else 0.001

print(f"Estimated STL height: {estimated_height_mm:.1f} mm", flush=True)
print(f"Target robot height: {target_height_m} m", flush=True)
print(f"Recommended scale factor: {scale_factor:.6f}", flush=True)
print(f"  (Multiply STL coordinates by this to get meters)", flush=True)

# Also check body part positions for joint placement
print("\n" + "="*60, flush=True)
print("JOINT POSITION ESTIMATES (at scale)", flush=True)
print("="*60, flush=True)

# Read specific parts for joint positions
hip_min, hip_max = get_stl_bounds_via_numpy(f"{PART2}/AT-ST_Legs_Hip_x2.stl")
upper_min, upper_max = get_stl_bounds_via_numpy(f"{PART2}/AT-ST_Legs_Upper_left_a_x1.stl")
foot_min, foot_max = get_stl_bounds_via_numpy(f"{PART2}/AT-ST_Foot_left__x1.stl")

if hip_min is not None:
    hip_center_z = (hip_min[2] + hip_max[2]) / 2 * scale_factor
    print(f"Hip joint Z (m): ~{hip_center_z:.3f}", flush=True)

if upper_min is not None:
    upper_top_z = upper_max[2] * scale_factor
    upper_bottom_z = upper_min[2] * scale_factor
    print(f"Upper leg top Z (m): ~{upper_top_z:.3f}", flush=True)
    print(f"Upper leg bottom Z (m): ~{upper_bottom_z:.3f}", flush=True)

if foot_min is not None:
    foot_bottom_z = foot_min[2] * scale_factor
    foot_height = (foot_max[2] - foot_min[2]) * scale_factor
    print(f"Foot bottom Z (m): ~{foot_bottom_z:.3f}", flush=True)
    print(f"Foot height (m): ~{foot_height:.3f}", flush=True)

print("\n" + "="*60, flush=True)
print("ANALYSIS COMPLETE", flush=True)
print("="*60, flush=True)
print(f"\nRecommended next steps:", flush=True)
print(f"1. Use scale factor {scale_factor:.6f} when importing STL meshes", flush=True)
print(f"2. Adjust physics skeleton to match scaled dimensions", flush=True)
print(f"3. Combine visual meshes with physics bodies", flush=True)

simulation_app.close()
print("Done!", flush=True)
