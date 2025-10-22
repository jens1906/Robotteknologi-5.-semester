import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import time
import pyrealsense2 as rs
import open3d as o3d


Test=False
VideoTest=True
start_time = time.time()
image_path = 'Vision\TestData\Blob_1_color.png'
depth = np.load('Vision\TestData\Blob_1_depth.npy')
image = cv.imread(image_path)

if image is None:
    raise ValueError("Image not found or unable to load.")

kernel = np.ones((5, 5), np.uint8)

# For mm transformation
F = 1.93
PixelSize = 0.003
F_ideal = F / PixelSize
h, w, _ = image.shape
fx, fy = F_ideal, F_ideal
cx = w / 2
cy = h / 2

# Realtime setup of RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

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

    _, thresh_h = cv.threshold(h, 250, 255, cv.THRESH_BINARY)
    _, thresh_s = cv.threshold(s, 75, 255, cv.THRESH_BINARY)
    _, thresh_v = cv.threshold(v, 25, 255, cv.THRESH_BINARY)

    thresh_hsv = cv.merge([thresh_h, thresh_s, thresh_v])
    thresh_hsv = cv.cvtColor(thresh_hsv, cv.COLOR_HSV2BGR)
    return thresh_hsv


def clean_image(image):
    #Split image into red, green, blue channels and theshold red channel

    img_erode = cv.erode(image, kernel, iterations=1)
    img_dilate = cv.dilate(img_erode, kernel, iterations=3)
    img_final_erode = cv.erode(img_dilate, kernel, iterations=3)
    if Test: 
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

def combine_and_transform(scatter_data, depth, fx, fy, cx, cy):
    depth_values = depth[scatter_data[:, 1], scatter_data[:, 0]]
    xyz_data = np.column_stack(((scatter_data[:, 0] - cx) * depth_values / fx, 
                                (scatter_data[:, 1] - cy) * depth_values / fy, 
                                 depth_values))
    if Test:
        plot_and_print_results(scatter_data, depth_values, xyz_data)
        # Output point cloud to CSV for RoboDK
        np.savetxt('point_cloud_xyz_mm.csv', xyz_data, delimiter=',', header='X,Y,Z', comments='', fmt='%.6f')

    return xyz_data



try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        xyz_data = combine_and_transform(edge_to_scatter_plot(color_image), depth_image, fx, fy, cx, cy)

        pcd.points = o3d.utility.Vector3dVector(xyz_data[::2])  # Show every 2nd point
        
        if VideoTest:
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

end_time = time.time()
print(f"Total execution time: {end_time - start_time:.2f} seconds")



