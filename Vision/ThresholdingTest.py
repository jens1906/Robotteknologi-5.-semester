# Thresholding of an image (robust version)
import os
import cv2 as cv
import numpy as np

# Build a portable path to the image
img_path = os.path.join('Vision', 'TestData', 'RandomImage.jpg')
img = cv.imread(img_path)
if img is None:
	raise FileNotFoundError(f"Could not load image at '{img_path}'. Verify the file exists and the working directory is correct (cwd={os.getcwd()}).")

# Convert to HSV and split channels
hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
h, s, v = cv.split(hsv)

# Apply thresholds. cv.threshold returns (retval, thresh_image)
_, thresh_h = cv.threshold(h, 250, 255, cv.THRESH_BINARY)
_, thresh_s = cv.threshold(s, 75, 255, cv.THRESH_BINARY)
_, thresh_v = cv.threshold(v, 25, 255, cv.THRESH_BINARY)

# If the H-threshold is outside the H channel range in OpenCV (0-179), warn the user
if np.max(h) < 250:
	print(f"Warning: H channel max is {int(np.max(h))}; threshold=250 will produce an empty mask. Consider using a value in 0-179 for the H channel.")

# Combine thresholded channels (logical AND)
thresh_hsv = cv.merge([thresh_h, thresh_s, thresh_v])
thresh_hsv = cv.cvtColor(thresh_hsv, cv.COLOR_HSV2BGR)

kernel = np.ones((5, 5), np.uint8)
img_erode = cv.erode(thresh_hsv, kernel, iterations=8)
img_dilate = cv.dilate(img_erode, kernel, iterations=15)
img_final_erode = cv.erode(img_dilate, kernel, iterations=8)




# Resize for display if you want a smaller window
scale = 0.1  # 0.25 smaller, 1.0 original
if scale <= 0:
	raise ValueError("scale must be > 0")
thresh_small = cv.resize(img_final_erode, (0, 0), fx=scale, fy=scale, interpolation=cv.INTER_NEAREST)

cv.imshow("Thresholded HSV", thresh_small)
cv.waitKey(0)
cv.destroyAllWindows()