"""
Save RGB Image and Depth Array from Intel RealSense D435
==========================================================

This script captures one frame from the RealSense D435 and saves:
1. RGB color image as PNG file
2. Depth array as numpy .npy file (and optional .txt/.csv)

Files saved:
- color_image.png - RGB color image
- depth_array.npy - Depth data (numpy format, best for loading back)
- depth_array.txt - Depth data (text format, human readable)
- depth_array.csv - Depth data (CSV format, can open in Excel)

Usage:
python save_realsense_data.py

"""

import pyrealsense2 as rs
import numpy as np
import cv2
import sys
from datetime import datetime

# ============================================================================
# CONFIGURATION
# ============================================================================
WIDTH = 640
HEIGHT = 480
FPS = 30

# File name prefix (timestamp will be added)
SAVE_PREFIX = "realsense_capture"

# What formats to save depth data in
SAVE_NPY = True   # Numpy format (recommended, small, fast)
SAVE_TXT = False  # Text format (huge file, slow)
SAVE_CSV = False  # CSV format (huge file, can open in Excel)

# ============================================================================

def main():
    print("=" * 60)
    print("RealSense D435 - Save RGB Image + Depth Array")
    print("=" * 60)
    
    # Connect to RealSense device
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("❌ No RealSense devices found")
        sys.exit(1)

    dev = devices[0]
    print(f"✓ Found device: {dev.get_info(rs.camera_info.name)}\n")

    # Configure pipeline for BOTH color and depth
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Enable depth stream
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    
    # Enable color stream
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    
    print(f"✓ Configuring streams: {WIDTH}x{HEIGHT} @ {FPS}fps")
    
    # Start streaming
    pipeline.start(config)
    print("✓ Pipeline started\n")
    
    try:
        print("Waiting for frames to stabilize...")
        
        # Let auto-exposure stabilize (skip first few frames)
        for _ in range(30):
            pipeline.wait_for_frames()
        
        print("✓ Camera ready, capturing...\n")
        
        # Capture frames
        frames = pipeline.wait_for_frames(timeout_ms=5000)
        
        # Get depth frame
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            print("❌ No depth frame received")
            return
        
        # Get color frame
        color_frame = frames.get_color_frame()
        if not color_frame:
            print("❌ No color frame received")
            return
        
        # Convert to numpy arrays
        depth_array = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Create timestamp for unique filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # ===== SAVE COLOR IMAGE =====
        color_filename = f"{SAVE_PREFIX}_{timestamp}_color.png"
        cv2.imwrite(color_filename, color_image)
        print(f"✓ Saved color image: {color_filename}")
        print(f"  Image shape: {color_image.shape} (Height x Width x Channels)")
        print(f"  Image size: {color_image.nbytes / 1024:.1f} KB\n")
        
        # ===== SAVE DEPTH ARRAY =====
        print(f"Depth Array Info:")
        print(f"  Shape: {depth_array.shape} (Height x Width)")
        print(f"  Data type: {depth_array.dtype}")
        valid_depths = depth_array[depth_array > 0]
        if len(valid_depths) > 0:
            print(f"  Min depth: {valid_depths.min()}mm ({valid_depths.min()/1000:.2f}m)")
            print(f"  Max depth: {valid_depths.max()}mm ({valid_depths.max()/1000:.2f}m)")
            print(f"  Valid pixels: {len(valid_depths)}/{depth_array.size} ({100*len(valid_depths)/depth_array.size:.1f}%)")
        print()
        
        # Save as numpy file (.npy) - RECOMMENDED
        if SAVE_NPY:
            npy_filename = f"{SAVE_PREFIX}_{timestamp}_depth.npy"
            np.save(npy_filename, depth_array)
            file_size = np.load(npy_filename, mmap_mode='r').nbytes / 1024
            print(f"✓ Saved depth as numpy: {npy_filename}")
            print(f"  File size: {file_size:.1f} KB")
            print(f"  To load: depth = np.load('{npy_filename}')\n")
        
        # Save as text file (.txt) - OPTIONAL
        if SAVE_TXT:
            txt_filename = f"{SAVE_PREFIX}_{timestamp}_depth.txt"
            print(f"⏳ Saving as text file (this may take a while)...")
            np.savetxt(txt_filename, depth_array, fmt='%d')
            import os
            file_size = os.path.getsize(txt_filename) / (1024 * 1024)
            print(f"✓ Saved depth as text: {txt_filename}")
            print(f"  File size: {file_size:.1f} MB (WARNING: Large!)\n")
        
        # Save as CSV file (.csv) - OPTIONAL
        if SAVE_CSV:
            csv_filename = f"{SAVE_PREFIX}_{timestamp}_depth.csv"
            print(f"⏳ Saving as CSV file (this may take a while)...")
            np.savetxt(csv_filename, depth_array, fmt='%d', delimiter=',')
            import os
            file_size = os.path.getsize(csv_filename) / (1024 * 1024)
            print(f"✓ Saved depth as CSV: {csv_filename}")
            print(f"  File size: {file_size:.1f} MB (can open in Excel)\n")
        
        print("=" * 60)
        print("✓ ALL FILES SAVED SUCCESSFULLY!")
        print("=" * 60)
        
    except Exception as e:
        print(f"❌ Error: {e}")
    
    finally:
        pipeline.stop()
        print("\n✓ Pipeline stopped")


# Run main function 5 times with 1 second interval
for _ in range(10):
    main()
    import time
    time.sleep(0.2)

