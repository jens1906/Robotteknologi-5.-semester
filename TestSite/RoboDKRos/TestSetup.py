from pathlib import Path
from robodk import robolink, robomath

def main():
    RDK = robolink.Robolink()  # Connect to RoboDK
    RDK.setRunMode(robolink.RUNMODE_SIMULATE)
    RDK.setSimulationSpeed(10)

    # Detect project root automatically (2 levels up from this file)
    project_root = Path(__file__).resolve().parents[2]
    testsite_dir = project_root / "TestSite"
    robodkros_dir = testsite_dir / "RoboDKRos"

    print(f"📂 Using project root: {project_root}")

    # --- Frame 1 ---
    frame1 = RDK.AddFrame("Frame 1")
    frame1.setPose(robomath.eye(4))  # World origin

    # --- Bordplade ---
    bordplade_path = Path(__file__).parent / "Bundplade.stl"
    if bordplade_path.exists():
        bordplade = RDK.AddFile(str(bordplade_path))
        bordplade.setParent(frame1)
        bordplade.setPose(robomath.transl(0, 0, 0))
    else:
        print(f"⚠️ Bordplade not found at: {bordplade_path}")

    # --- Frame 2 ---
    frame2 = RDK.AddFrame("Frame 2", frame1)
    frame2.setPose(robomath.transl(200, 0, 250))

    # --- Mount ---
    mount_path = Path(__file__).parent / "UR3e mount.stl"
    if mount_path.exists():
        mount = RDK.AddFile(str(mount_path))
        mount.setParent(frame2)
        mount.setPose(robomath.transl(0, 0, 0))
    else:
        print(f"⚠️ Mount not found at: {mount_path}")

    # --- UR3e Base ---
    ur3e_base = RDK.AddFrame("Frame 3", frame2)
    pose_ur3e_base = robomath.Mat([
        [1, 0, 0, 0],
        [0, 0, -1, -218],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    ur3e_base.setPose(pose_ur3e_base)

    # --- UR3e Robot ---
    ur_path = testsite_dir / "UR3e.robot"
    if ur_path.exists():
        ur_robot = RDK.AddFile(str(ur_path))
        if ur3e_base.Valid() and ur_robot.Valid():
            ur_robot.setParent(ur3e_base)
            pose_robot = robomath.Mat([
                [1, 0, 0, 200],
                [0, 0, -1, 0],
                [0, 1, 0, -380],
                [0, 0, 0, 1]
            ])
            ur_robot.setPose(pose_robot)
        else:
            print("⚠️ Robot or UR3e Base frame invalid, skipping parenting.")
    else:
        print(f"⚠️ UR3e robot not found at: {ur_path}")

    # --- Frame 4 ---
    frame4 = RDK.AddFrame("Frame 4", frame1)
    frame4.setPose(robomath.Mat([
        [1, 0, 0, 200],
        [0, 0, -1, 0],
        [0, 1, 0, -380],
        [0, 0, 0, 1]
    ]))

    # --- TestpladeKurve ---
    testplade_path = robodkros_dir / "TestpladeKurve v3.stl"
    if testplade_path.exists():
        testplade = RDK.AddFile(str(testplade_path))
        testplade.setParent(frame4)
        pose_testplade = robomath.Mat([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -2973.5],
            [0, 0, 0, 1]
        ])
        testplade.setPose(pose_testplade)
    else:
        print(f"⚠️ TestpladeKurve file not found at: {testplade_path}")
    
    # --- Realsense Holder ---
    realsense_path = Path(__file__).parent / "RealsenseHolder.stl"
    if realsense_path.exists() and ur_path.exists():
        # Create a tool reference on the robot flange
        tool_frame = RDK.AddFrame("RealsenseHolder", ur_robot)
        tool_frame.setParent(ur_robot)

        # Position the tool frame at the robot’s TCP
        tool_pose = robomath.Mat([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -25],
            [0, 0, 0, 1]
        ])
        tool_frame.setPose(tool_pose)

        # Load the STL and attach it to the tool
        realsense_holder = RDK.AddFile(str(realsense_path))
        realsense_holder.setParent(tool_frame)

        # Adjust placement relative to TCP if needed
        realsense_holder.setPose(robomath.transl(0, 0, -65))
        
        print("✅ RealsenseHolder added and attached to UR3e end-effector.")

        # --- RSD435 Camera ---
        rsd435_path = Path(__file__).parent / "RSD435.stl"
        if rsd435_path.exists():
            rsd435 = RDK.AddFile(str(rsd435_path))
            rsd435.setParent(realsense_holder)

            # Adjust relative position/orientation as needed
            rsd435_pose = robomath.Mat([
                [-1, 0, 0, 0],     # Rotate/translate to align properly
                [0, -1, 0, 44],
                [0, 0, 1, 75],    # Move camera slightly in front of holder
                [0, 0, 0, 1]
            ])
            rsd435.setPose(rsd435_pose)

            # ✅ Disable collisions between:
            RDK.setCollisionActivePair(robolink.COLLISION_OFF, realsense_holder, rsd435)         # Holder ↔ Camera
            RDK.setCollisionActivePair(robolink.COLLISION_OFF, ur_robot, realsense_holder, id1=6) # UR3e J6 ↔ Holder
            RDK.setCollisionActivePair(robolink.COLLISION_OFF, ur_robot, rsd435, id1=6)           # UR3e J6 ↔ Camera

            print("✅ RSD435 camera added and attached to RealsenseHolder.")

            # --- Add Simulated 2D Camera View ---
            cam_params = (
                "SIZE=1920x1080 "               # Full rendering resolution
                "WINDOWSIZE=480x270 "           # Small visible window
                "MINIMIZED"
                "FOV=69 "
                "NEAR=100 FAR=10000 "
                "FOCAL_LENGTH=1.93 "
                "PIXELSIZE=1.4 "
                "PROJECTION=PERSP "
                "BG_COLOR=0xFF5078B0 "          # RoboDK sky blue tone
                "LIGHT_AMBIENT=0xFF7FA6D9 "
                "LIGHT_DIFFUSE=0xFF9CC2FF "
                "LIGHT_SPECULAR=0xFFBBD7FF "
            )

            cam_handle = RDK.Cam2D_Add(rsd435, cam_params)

        else:
            print(f"⚠️ RSD435 file not found at: {rsd435_path}")



if __name__ == "__main__":
    main()
