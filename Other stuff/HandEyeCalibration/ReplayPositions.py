# Replay Robot Positions from JSON Files
# This script reads the saved JSON files and moves the robot to those exact positions
# Useful for recapturing images with a different calibration board

from robodk import robolink, robomath
from pathlib import Path
import json
import time

RDK = robolink.Robolink()

# Get the robot
robot = RDK.Item('', robolink.ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception("No robot found in station")

# Check robot connection and run mode
print(f"Robot: {robot.Name()}")
connection_status = robot.ConnectedState()
print(f"Connection status code: {connection_status}")

# Connection status codes:
# ROBOTCOM_READY = 1 (ready to move)
# ROBOTCOM_WORKING = 2 (moving)
# ROBOTCOM_DISCONNECTED = 0 or other (not connected)

print("\nChoose run mode:")
print("1. REAL ROBOT - Execute on physical robot")
print("2. SIMULATION - Only simulate in RoboDK")
mode_choice = input("\nEnter choice (1 or 2): ").strip()

if mode_choice == '1':
    # Force run on real robot
    RDK.setRunMode(robolink.RUNMODE_RUN_ROBOT)
    print("✓ Run mode set to REAL ROBOT")
    print("  Robot will move in real life!")
    
    # Safety confirmation
    confirm = input("\n⚠ SAFETY CHECK: Robot will MOVE. Workspace clear? (yes/no): ").lower()
    if confirm != 'yes':
        print("Exiting for safety...")
        exit()
else:
    # Run in simulation
    RDK.setRunMode(robolink.RUNMODE_SIMULATE)
    print("✓ Run mode set to SIMULATION")
    print("  Robot will only move in RoboDK (not real life)")

# Get the Hand-Eye-Data folder
record_folder = Path(RDK.getParam(robolink.PATH_OPENSTATION)) / 'Hand-Eye-Data'

if not record_folder.exists():
    raise Exception(f"Hand-Eye-Data folder not found at: {record_folder}")

# Find all JSON files
json_files = sorted(record_folder.glob('*.json'), key=lambda x: int(x.stem) if x.stem.isdigit() else float('inf'))

if not json_files:
    raise Exception("No JSON files found in Hand-Eye-Data folder")

print("=" * 70)
print("REPLAY ROBOT POSITIONS FROM JSON FILES")
print("=" * 70)
print(f"Robot: {robot.Name()}")
print(f"Found {len(json_files)} saved positions")
print(f"Folder: {record_folder}")
print("=" * 70)

def capture_image():
    """Trigger image capture and wait for acknowledgment"""
    RDK.setParam('RECORD_READY', 1)
    
    timeout = 10
    start_time = time.time()
    while True:
        ack = RDK.getParam('RECORD_ACKNOWLEDGE')
        if ack == '1':
            RDK.setParam('RECORD_ACKNOWLEDGE', 0)
            return True
        
        if time.time() - start_time > timeout:
            return False
        
        time.sleep(0.1)

# Ask user if they want to delete old images
print("\nOptions:")
print("1. DELETE old images and start fresh (recommended)")
print("2. Keep old images and append new ones")
choice = input("\nEnter choice (1 or 2): ").strip()

if choice == '1':
    # Delete old PNG files
    png_files = list(record_folder.glob('*.png'))
    if png_files:
        confirm = input(f"\nThis will DELETE {len(png_files)} existing images. Continue? (yes/no): ").lower()
        if confirm == 'yes':
            for png_file in png_files:
                png_file.unlink()
            print(f"✓ Deleted {len(png_files)} old images")
        else:
            print("Cancelled")
            exit()

print("\n" + "=" * 70)
print("READY TO CAPTURE")
print("=" * 70)
print("Instructions:")
print("1. Make sure you have the CORRECT calibration board in place")
print("2. Make sure HandEyeAcquisition.py is running")
print("3. Press Enter to start capturing at all saved positions")
print("=" * 70)
input("\nPress Enter to begin...")

successful = 0
failed = 0

for i, json_file in enumerate(json_files):
    print(f"\n[{i+1}/{len(json_files)}] Loading position from {json_file.name}...")
    
    try:
        # Load the JSON file
        with open(json_file, 'r') as f:
            robot_data = json.load(f)
        
        # Get the joint values
        joints = robot_data.get('joints')
        if joints is None:
            print(f"  ✗ No 'joints' data in {json_file.name}, skipping...")
            failed += 1
            continue
        
        print(f"  → Moving to joints: {[f'{j:.1f}' for j in joints]}")
        
        # Move the robot to the exact position
        robot.MoveJ(joints)
        
        # Wait for stabilization
        time.sleep(1.5)
        
        # Capture image
        print(f"  📸 Capturing image...")
        if capture_image():
            print(f"  ✓ Image {i} captured successfully!")
            successful += 1
        else:
            print(f"  ✗ Failed to capture image {i}")
            print(f"     Is HandEyeAcquisition.py running?")
            failed += 1
            
    except Exception as e:
        print(f"  ✗ Error: {e}")
        failed += 1

# Stop acquisition script
print("\n" + "-" * 70)
print("Stopping acquisition script...")
RDK.setParam('RECORD_STOP', 1)
time.sleep(0.5)

# Summary
print("\n" + "=" * 70)
print("RECAPTURE COMPLETE!")
print("=" * 70)
print(f"Successful captures: {successful}")
print(f"Failed captures: {failed}")
print(f"Total positions: {len(json_files)}")
print("\nNext steps:")
print("1. Check the captured images in Hand-Eye-Data folder")
print("2. Make sure they show the CORRECT calibration board")
print("3. Run HandEyeCalibration.py to compute camera pose")
print("=" * 70)
