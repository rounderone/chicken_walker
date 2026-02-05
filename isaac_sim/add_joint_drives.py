# Add joint drives to Cassie to make it stand
import omni.usd
from pxr import UsdPhysics, PhysxSchema, Gf

stage = omni.usd.get_context().get_stage()

# Joint drive settings - high stiffness to hold position
STIFFNESS = 1000.0  # N*m/rad - spring stiffness
DAMPING = 100.0     # N*m*s/rad - damping

joints_configured = 0

for prim in stage.Traverse():
    p = prim.GetPath().pathString
    if "/World/Cassie/joints/" in p and prim.GetTypeName() == "PhysicsRevoluteJoint":
        joint_name = p.split("/")[-1]

        # Add drive to the joint
        # PhysX uses angular drive for revolute joints
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")

        # Set drive properties
        drive.CreateTypeAttr("force")  # force-based drive
        drive.CreateStiffnessAttr(STIFFNESS)
        drive.CreateDampingAttr(DAMPING)
        drive.CreateMaxForceAttr(10000.0)  # max torque

        # Set target to current position (0 = neutral)
        drive.CreateTargetPositionAttr(0.0)

        joints_configured += 1
        print(f"  Added drive to {joint_name}")

print(f"\nConfigured {joints_configured} joint drives")
print("Stiffness:", STIFFNESS, "N*m/rad")
print("Damping:", DAMPING, "N*m*s/rad")
print("\nPress STOP then PLAY to test - Cassie should now hold position!")
