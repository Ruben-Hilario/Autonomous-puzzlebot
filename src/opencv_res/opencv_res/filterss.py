import cv2 
import numpy as np
#from matplotlib import pylot as plt
target_height = target_width = 640
prob = 0.02

def roi_(resized_image,psc):
	roi = resized_image[int(target_height * psc):, :]
	a, b = roi.shape[:2]
	return roi,a,b

def edge_detection(resized_image):
    #ROI to grayscale and blur to reduce noise
    roi,_,_ = roi_(resized_image,0.75)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlue(gray,(5,5),1.4)

    # Threshold to binary inverse image (high contrast for edges)
    _, binary_inv = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)
    
    # Apply morphological operations to enhance features
    kernel = np.ones((3, 3), np.uint8)
    morph = cv2.erode(binary_inv, kernel, iterations=3)
    morph = cv2.dilate(morph, kernel, iterations=3)

    #Vertical sum
    detection = np.sum(morph, axis=0, dtype=np.float32)
    # Compute first and second gradients
    first_grad = np.gradient(detection)
    second_grad = np.gradient(first_grad)

    # Set dynamic thresholds to filter out insignificant edges
    line_thresh = 0.1
    thresh_pos = np.mean(first_grad) + line_thresh * (np.max(first_grad) - np.min(first_grad))
    thresh_neg = np.mean(first_grad) - line_thresh * (np.max(first_grad) - np.min(first_grad))
    
    # Identify potential left and right edges based on thresholds
    right_grad = np.where(first_grad > thresh_pos, second_grad, 0)
    left_grad = np.where(first_grad < thresh_neg, second_grad, 0)
    
    # Zero negative gradient values (removing false edges)
    right_grad[right_grad < 0] = 0
    left_grad[left_grad < 0] = 0
    
    # Detect edge positions by comparing with shifted gradients
    left_shift = np.roll(left_grad, 1)
    right_shift = np.roll(right_grad, 1)
    left_shift[0], right_shift[0] = 0, 0
    
    left_edges = np.argwhere(left_shift < left_grad).flatten()
    right_edges = np.argwhere(right_shift < right_grad).flatten()


    # Visualization: Image processing steps
    # plt.figure(figsize=(14, 8))
    # titles = ['Original', 'Resized', 'Gray (ROI)', 'Blurred', 'Binary Threshold', 'Morphological Result']
    images = [img, resized_img, gray, blurred, binary_inv, morph]
   
    return images
    
def line_detection_moments(resized_image):
	psc = 0.4
	roi, roi_height, roi_width = roi_(resized_image,psc)
	top_width = int(roi_width * 0.9)
	trapezoid = np.array([[((roi_width - top_width) // 2, 0),
	                       ((roi_width + top_width) // 2, 0),
	                       (roi_width, roi_height),
	                       (0, roi_height)]], dtype=np.int32)
	mask = np.zeros((roi_height, roi_width), dtype=np.uint8)
	cv2.fillPoly(mask, trapezoid, 255)
	
	# Apply the mask
	roi_masked = cv2.bitwise_and(roi, roi, mask=mask)
	
	# Convert to grayscale and apply thresholding
	gray = cv2.cvtColor(roi_masked, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	_, binary_inv = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)
	
	# Morphological operations
	kernel = np.ones((3, 3), np.uint8)
	morph = cv2.erode(binary_inv, kernel, iterations=3)
	morph = cv2.dilate(morph, kernel, iterations=3)
	
	# Connected Components
	num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(morph, connectivity=4)
	
	output = roi.copy()
	total_weight = 0
	weighted_sum = np.array([0.0, 0.0])
	MIN_AREA, MAX_AREA = 1000, 100000
	
	for i in range(1, num_labels):  # skip background
		x, y, w, h, area = stats[i]
		cx, cy = centroids[i]
	
		if MIN_AREA <= area <= MAX_AREA:
			cv2.rectangle(output, (x, y), (x + w, y + h), (255, 0, 0), 2)
			cv2.circle(output, (int(cx), int(cy)), 4, (0, 0, 255), -1)
			cv2.putText(output, f"({int(cx)},{int(cy)})", (int(cx) + 5, int(cy)),
	                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
	
			weighted_sum += np.array([cx, cy]) * area
			total_weight += area
	
	# Compute weighted average center (setpoint)
	if total_weight > 0:
		avg_cx, avg_cy = (weighted_sum / total_weight).astype(int)
		cv2.circle(output, (avg_cx, avg_cy), 6, (0, 255, 255), -1)
		cv2.putText(output, "Setpoint", (avg_cx + 10, avg_cy),
	                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
	

	return avg_cx, avg_cy, cv2.cvtColor(output, cv2.COLOR_BGR2RGB)
	
def line_detection_contours(resized_image):
	roi, roi_height, roi_width = roi_(resized_image,0.5)
	mask = np.zeros((roi_height, roi_width), dtype=np.uint8)
		
	top_width = int(roi_width * 0.9)
	trapezoid = np.array([[
	    ((roi_width - top_width) // 2, int(roi_height * 0.0)),  # top-left
	    ((roi_width + top_width) // 2, int(roi_height * 0.0)),  # top-right
	    (roi_width, roi_height),                                # bottom-right
	    (0, roi_height)                                            # bottom-left
	]], dtype=np.int32)
	
	cv2.fillPoly(mask, trapezoid, 255)
	roi = cv2.bitwise_and(roi, roi, mask=mask)
	
	# --- Step 2: Preprocess image ---
	gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	_, binary_inv = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)
	
	# Morphological operations
	kernel = np.ones((3, 3), np.uint8)
	morph = cv2.erode(binary_inv, kernel, iterations=3)
	morph = cv2.dilate(morph, kernel, iterations=3)
	
	# Canny edge detection
	canny_edges = cv2.Canny(morph, 50, 150)
	
	side_crop_percent = 0.05
	crop_x = int(roi_width * side_crop_percent)
	
	# Create a mask that only keeps the center part
	side_mask = np.zeros_like(canny_edges)
	cv2.rectangle(
	    side_mask,
	    (crop_x, 0),  # top-left corner
	    (roi_width - crop_x, roi_height),  # bottom-right corner
	    255,  # white region
	    thickness=-1
	)
	
	# Apply side mask
	canny_edges = cv2.bitwise_and(canny_edges, canny_edges, mask=side_mask)
	
	# Apply trapezoidal mask to Canny output
	all_edges_roi = cv2.bitwise_and(canny_edges, canny_edges, mask=mask)
	
	# --- Step 3: Find and process contours ---
	contours, _ = cv2.findContours(all_edges_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	
	output = roi.copy()
	epsilon_factor = 0.1
	for cnt in contours:
	    # Approximate polygon
		epsilon = epsilon_factor * cv2.arcLength(cnt, True)
		approx = cv2.approxPolyDP(cnt, epsilon, True)
	
	    # Bounding rectangle and center
		x, y, w, h = cv2.boundingRect(approx)
		cv2.rectangle(output, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Blue box
	
		box_area = w * h
	
	    # Filter by bounding box area
		if box_area < 1000 or box_area > 100000:
			continue
	
		cx = x + w // 2
		cy = y + h // 2
		cv2.circle(output, (cx, cy), 4, (0, 0, 255), -1)               # Red center
		cv2.putText(output, f"({cx},{cy})", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
	
	    # Draw the approximated polygon
		cv2.polylines(output, [approx], isClosed=True, color=(0, 255, 0), thickness=2)
	return 0
			
def line_detection_hugh(resized_image):
	roi, roi_height, roi_width = roi_(resized_image,0.4)
	# Create trapezoidal mask
	top_width = int(roi_width * 0.9)
	trapezoid = np.array([[((roi_width - top_width) // 2, 0),
	                       ((roi_width + top_width) // 2, 0),
	                       (roi_width, roi_height),
	                       (0, roi_height)]], dtype=np.int32)
	mask = np.zeros((roi_height, roi_width), dtype=np.uint8)
	cv2.fillPoly(mask, trapezoid, 255)
	
	# Apply the mask
	roi_masked = cv2.bitwise_and(roi, roi, mask=mask)
	
	# Convert to grayscale and apply thresholding
	gray = cv2.cvtColor(roi_masked, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	_, binary_inv = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)
	
	# Morphological operations
	kernel = np.ones((3, 3), np.uint8)
	morph = cv2.erode(binary_inv, kernel, iterations=3)
	morph = cv2.dilate(morph, kernel, iterations=3)
	
	   # Canny edge detection
	canny_edges = cv2.Canny(morph, 50, 150)
	
	    # Side crop mask
	CROP_SIDE_PERCENT = 0.05
	crop_x = int(roi_width * CROP_SIDE_PERCENT)
	side_mask = np.zeros_like(canny_edges)
	cv2.rectangle(side_mask, (crop_x, 0), (roi_width - crop_x, roi_height), 255, thickness=-1)
	canny_edges = cv2.bitwise_and(canny_edges, canny_edges, mask=side_mask)
	
	# Final mask with trapezoid
	edges_roi = cv2.bitwise_and(canny_edges, canny_edges, mask=mask)
	
	# Detect lines using HoughLinesP
	lines = cv2.HoughLinesP(
	    edges_roi,
	    rho=1,
	    theta=np.pi / 180,
	    threshold=50,
	    minLineLength=5,
	    maxLineGap=50
	)
	
	output = roi.copy()
	if lines is not None:
		for line in lines:
			x1, y1, x2, y2 = line[0]
			cv2.line(output, (x1, y1), (x2, y2), (0, 255, 0), 3)
	return output, canny_edges

#------------------------------#
		#Preprocessing#
#------------------------------#
def sharpened_filter(resized_image):
	resized_image = cv2.cvtColor(resized_image,cv2.COLOR_BGR2RGB)
	# Define sharpening kernel
	sharpen_kernel = np.array([[-1, -1, -1],
	                           [-1,  9, -1],
	                           [-1, -1, -1]])
	
	# Apply sharpening filter using filter2D
	sharpened_img = cv2.filter2D(resized_image, -1, sharpen_kernel)
	return sharpened_img

def add_salt_pepper_noise(image):
    noisy = image.copy()
    total_pixels = image.size // 3  # For RGB
    num_salt = int(prob * total_pixels / 2)
    num_pepper = int(prob * total_pixels / 2)

    # Add white (salt) pixels
    coords = [np.random.randint(0, i - 1, num_salt) for i in image.shape[:2]]
    noisy[coords[0], coords[1]] = [255, 255, 255]

    # Add black (pepper) pixels
    coords = [np.random.randint(0, i - 1, num_pepper) for i in image.shape[:2]]
    noisy[coords[0], coords[1]] = [0, 0, 0]

    return noisy

def add_gaussian_noise(image, mean=0, std=25):
    noise = np.random.normal(mean, std, image.shape).astype(np.float32)
    noisy = image.astype(np.float32) + noise
    noisy = np.clip(noisy, 0, 255).astype(np.uint8)
    return noisy
