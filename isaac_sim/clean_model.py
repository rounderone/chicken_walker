"""
Clean AT-ST model - remove problematic variant references
Run with: C:\IS\python.bat D:\Projects\ATST\isaac_sim\clean_model.py
"""
print("=" * 60)
print("Cleaning AT-ST model")
print("=" * 60)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Sdf

# Open the original file
print("\n[1] Opening original model...")
stage = Usd.Stage.Open("D:/Projects/ATST/usd/atst.usdc")

if not stage:
    print("ERROR: Could not open stage")
    exit(1)

# Get root prim
root = stage.GetDefaultPrim()
print(f"    Default prim: {root.GetPath() if root else 'None'}")

# Check for variant sets
print("\n[2] Checking for variant sets...")
for prim in stage.Traverse():
    vs = prim.GetVariantSets()
    names = vs.GetNames()
    if names:
        print(f"    {prim.GetPath()} has variants: {names}")
        for name in names:
            vset = vs.GetVariantSet(name)
            selections = vset.GetVariantNames()
            current = vset.GetVariantSelection()
            print(f"      - {name}: options={selections}, current='{current}'")

# Check prim count
prim_count = len(list(stage.Traverse()))
print(f"\n[3] Total prims: {prim_count}")

# Find the atst_walker prim
atst = stage.GetPrimAtPath("/atst_walker")
if atst:
    print(f"\n[4] AT-ST prim found: {atst.GetPath()}")

    # Clear variant selections to use base/none
    vs = atst.GetVariantSets()
    for name in vs.GetNames():
        vset = vs.GetVariantSet(name)
        # Set to empty or first available that doesn't need external file
        variants = vset.GetVariantNames()
        print(f"    Clearing variant '{name}' (was '{vset.GetVariantSelection()}')")
        if "None" in variants:
            vset.SetVariantSelection("None")
        elif "" in variants:
            vset.SetVariantSelection("")
        else:
            vset.ClearVariantSelection()

# Export as USDA (text format) for easier debugging
output = "D:/Projects/ATST/usd/atst_clean.usda"
print(f"\n[5] Exporting to {output}...")
stage.Export(output)

print("\n" + "=" * 60)
print("Done! Check the cleaned file:")
print(f"  {output}")
print("=" * 60)

simulation_app.close()
