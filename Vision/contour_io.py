"""
Contour Input/Output Module
Handles saving and loading of contour boundary data in CSV format.
"""
import numpy as np
import csv


def save_contour_data(hull, expanded_hull, centroid, image_shape, filename="contour_data.csv"):
    """Save contour boundary data to a CSV file compatible with RoboDK."""
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Write header
        writer.writerow(['X', 'Y', 'Point_Index'])
        
        # Write only expanded hull points
        for i, point in enumerate(expanded_hull):
            writer.writerow([point[0][0], point[0][1], i+1])
    
    print(f"Contour data saved to {filename}")
    print(f"  - Expanded hull: {len(expanded_hull)} points")


def load_contour_data(filename="contour_data.csv"):
    """Load contour boundary data from a CSV file."""
    expanded_hull = []
    
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        
        for row in reader:
            x = int(row['X'])
            y = int(row['Y'])
            expanded_hull.append([[x, y]])
    
    # Convert to numpy array
    expanded_hull = np.array(expanded_hull, dtype=np.int32)
    
    print(f"Contour data loaded from {filename}")
    print(f"  - Expanded hull: {len(expanded_hull)} points")
    return expanded_hull
