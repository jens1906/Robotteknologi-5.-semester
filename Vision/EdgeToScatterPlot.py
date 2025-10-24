import glob
import os
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import time
import pyrealsense2 as rs
import open3d as o3d
from robodk.robolink import *      
from robodk.robomath import *     
from scipy.interpolate import splprep, splev


# Start the RoboDK API:
RDK = Robolink()

CameraTest=False
CameraTestVisuals=False
PictureTest=False
PictureTestVisuals=False
CaptionTest = True
SavePointCloud=True

Format = [640, 480]
kernel = np.ones((5, 5), np.uint8)

# For mm transformation
F = 1.93
PixelSize = 0.003
F_ideal = F / PixelSize
F = F_ideal, F_ideal
c = Format[1] / 2, Format[0] / 2


# Realtime setup of RealSense
if CameraTest:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, Format[0], Format[1], rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, Format[0], Format[1], rs.format.z16, 30)
    pipeline.start(config)

if CameraTest and CameraTestVisuals:
    # Open3D visualizer setup
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pcd = o3d.geometry.PointCloud()
    first_frame = True



'''
THIS IS JUST FOR TESTING PURPOSES
'''
def plot_dialation_erotion(img1, img2, img3, img4):
    fig, axes = plt.subplots(1, 4, figsize=(10, 5))
    for ax, img, title in zip(axes, [img1, img2, img3, img4], ['Original', 'Eroded', 'Dilated', 'Final Eroded']):
        ax.imshow(img, cmap='gray')
        ax.set_title(title)
        ax.axis('off')
    plt.tight_layout()
    plt.show()

def plot_and_print_results(scatter_data, depth_values, xyz_data):
    for i, label in enumerate(['Width', 'Height', 'Depth']):
        print(f"{label} of object (mm): {xyz_data[:, i].ptp():.2f}")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    for ax in (ax1, ax2):
        ax.scatter(scatter_data[:, 0], scatter_data[:, 1], 
                   c=depth_values if ax == ax2 else None, s=1, cmap='viridis')
        ax.invert_yaxis()
    plt.colorbar(ax2.collections[0], ax=ax2, label='Depth Value')
    plt.show()




'''
THIS IS THE MAIN CODE
'''

def threshold_corrosion(image):
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    h, s, v = cv.split(hsv)

    # ImageJ HSB uses H: 0-255, but OpenCV HSV uses H: 0-179
    # Convert ImageJ thresholds: OpenCV_H = ImageJ_H * (179/255)
    # ImageJ: H: 0-25, S: 75-255, B: 25-255
    h_min = int(0 * 179 / 255)    # 0 -> 0
    h_max = int(25 * 179 / 255)   # 25 -> 17
    
    thresh_h = cv.inRange(h, h_min, h_max)  # H between 0-17 (OpenCV scale)
    _, thresh_s = cv.threshold(s, 75, 255, cv.THRESH_BINARY)   # S > 75 (same scale)
    _, thresh_v = cv.threshold(v, 25, 255, cv.THRESH_BINARY)   # V > 25 (same scale)
    
    combined_mask = cv.bitwise_and(thresh_h, cv.bitwise_and(thresh_s, thresh_v))
    thresh_hsv = cv.merge([combined_mask, combined_mask, combined_mask])
    cv.imshow("Thresholded HSV", thresh_hsv)
    cv.waitKey(1)
    return thresh_hsv


def clean_image(image):
    img_erode = cv.erode(image, kernel, iterations=1)
    img_dilate = cv.dilate(img_erode, kernel, iterations=3)
    img_final_erode = cv.erode(img_dilate, kernel, iterations=3)
    if PictureTestVisuals: 
        plot_dialation_erotion(image, img_erode, img_dilate, img_final_erode)
    return img_final_erode

def edge_to_scatter_plot(image, threshold1=100, threshold2=200):
    img_final_erode = cv.cvtColor(clean_image(threshold_corrosion(image)), cv.COLOR_BGR2GRAY)

    edges = cv.Canny(img_final_erode, threshold1, threshold2)
    contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    filled_mask = np.zeros_like(img_final_erode)
    cv.drawContours(filled_mask, contours, -1, 255, thickness=cv.FILLED)

    y_indices, x_indices = np.where(filled_mask > 0)
    scatter_data = np.column_stack((x_indices, y_indices))
    return scatter_data

def clean_depth(area,depth):
    # If depth is within area and there is a fall out where it is zero, take average of nearby points which is not zero
    cleaned_depth = depth.copy()
    for point in area:
        x, y = point
        if cleaned_depth[y, x] == 0:
            # Get nearby points
            neighbors = cleaned_depth[max(0, y-1):y+2, max(0, x-1):x+2].flatten()
            non_zero_neighbors = neighbors[neighbors > 0]
            if non_zero_neighbors.size > 0:
                cleaned_depth[y, x] = np.mean(non_zero_neighbors)
    return cleaned_depth


def combine_and_transform(scatter_data, depth, depthFiles=None):
    if depth is None and depthFiles is not None:
        all_Depths = []
        for depth_file in depthFiles:
            depth = np.load(depth_file)
            cleaned_depth_image = clean_depth(scatter_data, depth)
            all_Depths.append(cleaned_depth_image)
            
        cleaned_depth_image = np.mean(all_Depths, axis=0)
        if PictureTestVisuals:
            plt.imshow(cleaned_depth_image, cmap='gray')
            plt.title('Averaged Cleaned Depth Image')
    else:
        cleaned_depth_image = clean_depth(scatter_data, depth)
    depth_values = cleaned_depth_image[scatter_data[:, 1], scatter_data[:, 0]]
    xyz_data = np.column_stack(((scatter_data[:, 0] - c[0]) * depth_values / F[0], 
                                (scatter_data[:, 1] - c[1]) * depth_values / F[1], 
                                 depth_values))
    if PictureTestVisuals:
        plot_and_print_results(scatter_data, depth_values, xyz_data)
        np.savetxt('point_cloud_xyz_mm.csv', xyz_data, delimiter=',', header='X,Y,Z', comments='', fmt='%.6f')

    return xyz_data

def import_object_to_robodk(object_name, xyz_data):
    existing = RDK.Item(object_name)
    if existing.Valid():
        existing.Delete()

    # Add as object (points or curve)
    #obj = RDK.AddPoints(xyz_data.tolist(), True)  # True = add as curve
    obj = RDK.AddCurve(xyz_data.tolist(), None, [1, 0, 0])
    obj.setName(object_name)

def RealsenseLiveFeed():
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            xyz_data = combine_and_transform(edge_to_scatter_plot(color_image), depth_image)

            pcd.points = o3d.utility.Vector3dVector(xyz_data[::2])  # Show every 2nd point
            
            if CameraTestVisuals:
                cv.imshow('Color', color_image)
                cv.imshow('Depth', depth_image)
                if first_frame:
                    vis.add_geometry(pcd)
                    first_frame = False
                else:
                    vis.update_geometry(pcd)
                vis.poll_events()
                vis.update_renderer()

            if cv.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv.destroyAllWindows()

def FrameTest(image, depth, depthFiles=None):
    xyz_data = combine_and_transform(edge_to_scatter_plot(image), depth, depthFiles)
    import_object_to_robodk("Corrosion_Object", xyz_data)
    #xyz_list = xyz_data[::5].tolist() 
    #curve = RDK.AddCurve(xyz_list)
    #curve.setName("SplinePath")
    #curve.setParam("Smooth", 1)
    if SavePointCloud:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz_data)
        o3d.io.write_point_cloud("point_cloud.pcd", pcd)  
        o3d.io.write_point_cloud("point_cloud.ply", pcd)
        np.savetxt('point_cloud.txt', xyz_data, delimiter=' ', fmt='%.6f')
        np.savetxt('point_cloud_xyz_mm.csv', xyz_data, delimiter=',', header='X,Y,Z', comments='', fmt='%.6f')

     
    



def main():
    if CameraTest:
        RealsenseLiveFeed()
    elif PictureTest:
        image_path = 'Vision\ImageTestData\ImageEarlyTestSingular\Blob_1_color.png'
        depth = np.load('Vision\ImageTestData\ImageEarlyTestSingular\Blob_1_depth.npy')
        image = cv.imread(image_path)

        if image is None:
            raise ValueError("Image not found or unable to load.")        
        FrameTest(image, depth)
    elif CaptionTest:
        #Make list of all images in a folder and a list of all depth files
        image_folder = 'Vision\ImageTestData\ImageCurveBad'
        depth_folder = 'Vision\ImageTestData\ImageCurveBad'
        image_files = glob.glob(os.path.join(image_folder, '*.png'))
        depth_files = glob.glob(os.path.join(depth_folder, '*.npy'))

        image = cv.imread(image_files[0])
        print(image_files)
        print(depth_files)

        FrameTest(image, None, depth_files)

    pass



main()

