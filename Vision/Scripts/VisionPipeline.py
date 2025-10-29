"""
Rust/Corrosion Detection Pipeline
Uses computer vision to detect and highlight rust areas in images.
"""
import cv2
import numpy as np
import os
from contour_io import save_contour_data, load_contour_data


# Configuration
IMAGE_PATH = "Vision/Corrosion2.png"
OUTPUT_PATH = r"Vision/contour_data.csv"  # Full path including filename
EXPANSION_PIXELS = 20  # How much to expand the boundary outward
MIN_RUST_AREA = 50     # Minimum pixel area to consider as rust


def show_image(window_name, image, wait=True):
    """Display an image in a window."""
    cv2.imshow(window_name, image)
    if wait:
        cv2.waitKey(0)

def enhance_red_channel(image):
    """Emphasize red areas in the image (rust is reddish-brown)."""
    blue = image[:, :, 0].astype(float)
    green = image[:, :, 1].astype(float)
    red = image[:, :, 2].astype(float)
    red_emphasis = np.clip(red - 0.5 * green - 0.5 * blue, 0, 255).astype(np.uint8)
    return red_emphasis


def preprocess_image(red_emphasis):
    """Enhance contrast and remove noise."""
    # Improve contrast
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    equalized = clahe.apply(red_emphasis)
    
    # Smooth while preserving edges
    smoothed = cv2.bilateralFilter(equalized, 9, 75, 75)
    return equalized, smoothed


def segment_rust(smoothed):
    """Create binary mask of rust regions."""
    # Convert to black and white
    binary_mask = cv2.adaptiveThreshold(smoothed, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                        cv2.THRESH_BINARY, 15, -8)
    
    # Clean up: remove noise and fill gaps
    kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    opened = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel_small, iterations=2)
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel_large, iterations=4)
    
    # Remove small blobs
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(closed, connectivity=8)
    cleaned = np.zeros_like(closed)
    for i in range(1, num_labels):
        if stats[i, cv2.CC_STAT_AREA] >= MIN_RUST_AREA:
            cleaned[labels == i] = 255
    
    return binary_mask, opened, closed, cleaned


def find_boundaries(mask, expansion_pixels):
    """Find tight and expanded boundaries around detected rust."""
    # Find all rust contours and combine them
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    all_points = np.vstack(contours)
    hull = cv2.convexHull(all_points)
    
    # Calculate centroid
    M = cv2.moments(hull)
    cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else hull[0][0][0]
    cy = int(M["m01"] / M["m00"]) if M["m00"] != 0 else hull[0][0][1]
    
    # Expand boundary outward from center
    expanded_hull = []
    for point in hull:
        px, py = point[0]
        dx, dy = px - cx, py - cy
        distance = np.sqrt(dx**2 + dy**2)
        if distance > 0:
            dx = dx / distance * expansion_pixels
            dy = dy / distance * expansion_pixels
            expanded_hull.append([[int(px + dx), int(py + dy)]])
    
    expanded_hull = np.array(expanded_hull, dtype=np.int32)
    return hull, expanded_hull, (cx, cy)


def create_grid_visualization(steps, target_height=300):
    """Create a grid layout of processing steps."""
    resized = []
    for img in steps:
        aspect_ratio = img.shape[1] / img.shape[0]
        target_width = int(target_height * aspect_ratio)
        resized.append(cv2.resize(img, (target_width, target_height)))
    
    # Create 2 rows of 5 columns
    row1 = np.hstack(resized[0:5])
    row2 = np.hstack(resized[5:10])
    return np.vstack([row1, row2])


def main():
    # Load image
    if not os.path.exists(IMAGE_PATH):
        print(f"Error: Image not found at {IMAGE_PATH}")
        return
    
    original = cv2.imread(IMAGE_PATH)
    if original is None:
        print("Error: Failed to load image.")
        return
    
    show_image("1. Original Image", original, wait=False)
    
    # Step 1: Emphasize red channel
    red_emphasis = enhance_red_channel(original)
    show_image("2. Red Channel Emphasis", red_emphasis, wait=False)
    
    # Step 2: Enhance contrast and denoise
    equalized, smoothed = preprocess_image(red_emphasis)
    show_image("3. Contrast Enhancement", equalized, wait=False)
    show_image("4. Smoothing and Denoising", smoothed, wait=False)
    
    # Step 3: Segment rust regions
    binary_mask, opened, closed, cleaned = segment_rust(smoothed)
    show_image("5. Binary Mask", binary_mask, wait=False)
    show_image("6. Noise Removal", opened, wait=False)
    show_image("7. Gap Filling", closed, wait=False)
    show_image("8. Small Blob Removal", cleaned, wait=False)
    
    # Step 4: Create visualization overlay
    overlay = original.copy()
    overlay[cleaned == 255] = [0, 0, 255]
    result = cv2.addWeighted(original, 0.7, overlay, 0.3, 0)
    show_image("9. Rust Detection Overlay", result, wait=False)
    
    # Step 5: Find and expand boundaries
    hull, expanded_hull, (cx, cy) = find_boundaries(cleaned, EXPANSION_PIXELS)
    save_contour_data(hull, expanded_hull, (cx, cy), original.shape, OUTPUT_PATH)
    
    # Draw boundaries on result
    result_with_contour = result.copy()
    cv2.drawContours(result_with_contour, [hull], 0, (255, 255, 0), 2)
    cv2.drawContours(result_with_contour, [expanded_hull], 0, (0, 255, 0), 3)
    cv2.circle(result_with_contour, (cx, cy), 5, (255, 0, 0), -1)
    show_image("10. Final Result with Boundaries", result_with_contour, wait=False)
    
    # Save the final image
    output_dir = os.path.dirname(OUTPUT_PATH)
    final_result_path = os.path.join(output_dir, "final_result_with_boundaries.jpg")
    cv2.imwrite(final_result_path, result_with_contour)
    print(f"Saved final result to {final_result_path}")
    
    # Create summary grids
    steps = [
        original,
        cv2.cvtColor(red_emphasis, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(equalized, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(smoothed, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(opened, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(closed, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR),
        result,
        result_with_contour
    ]
    
    grid_with_contour = create_grid_visualization(steps)
    grid_without_contour = create_grid_visualization(steps[:-1] + [np.zeros_like(steps[-1])])
    
    # Save the grid image
    grid_path = os.path.join(output_dir, "processing_steps_grid.jpg")
    cv2.imwrite(grid_path, grid_with_contour)
    print(f"Saved processing steps grid to {grid_path}")

    # Save the grid image without contour
    grid_without_contour_path = os.path.join(output_dir, "processing_steps_grid_without_contour.jpg")
    cv2.imwrite(grid_without_contour_path, grid_without_contour)
    print(f"Saved processing steps grid without contour to {grid_without_contour_path}")
    
    show_image("All Processing Steps (WITH Contour)", grid_with_contour, wait=False)
    show_image("All Processing Steps (WITHOUT Contour)", grid_without_contour, wait=False)
    
    print("\nPress any key to close all windows...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Verify saved data
    print("\n--- Verifying saved data ---")
    loaded_expanded = load_contour_data(OUTPUT_PATH)
    print(f"Loaded expanded hull points: {len(loaded_expanded)}")

if __name__ == "__main__":
    main()
