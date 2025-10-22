import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import time

Test=False
start_time = time.time()
image_path = 'Vision\TestData\Blob_1_color.png'
depth = np.load('Vision\TestData\Blob_1_depth.npy')
image = cv.imread(image_path)

if image is None:
    raise ValueError("Image not found or unable to load.")

kernel = np.ones((5, 5), np.uint8)
F = 1.93
PixelSize = 0.003
F_ideal = F / PixelSize
h, w, _ = image.shape
fx, fy = F_ideal, F_ideal
cx = w / 2
cy = h / 2

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

xyz_data = combine_and_transform(edge_to_scatter_plot(image), depth, fx, fy, cx, cy)
end_time = time.time()
print(f"Total execution time: {end_time - start_time:.2f} seconds")



