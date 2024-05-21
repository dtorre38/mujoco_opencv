import os
import cv2
import numpy as np


debug_opencv = 0  # set to 1 for saving opencv images for debugging purposes


def detect_and_draw_bound(frame, width=640, height=480):
    global bounding_box, dx, dy
    
    # Initialize bounding box coordinates with default values
    x, y, w, h = 0, 0, 0, 0
    
    # Reshape mujoco frame [height x width, 1] to 3D array for opencv input [height, width, 3]
    frame_reshaped = frame.reshape((height, width, 3))

    # convert frame to BGR - I think it's already coming in as BGR for some reason
    frame_bgr = cv2.cvtColor(frame_reshaped, cv2.COLOR_RGB2BGR)

    # Convert frame to HSV (Hue, Saturation, Value) color space for easier color detection
    hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    # Define range for color in HSV (link helps define range of values for colors)
    # https://stackoverflow.com/questions/47483951/how-can-i-define-a-threshold-value-to-detect-only-green-colour-objects-in-an-ima/47483966#47483966
    lower_color = np.array([20, 0, 0])  # yellow lower
    upper_color = np.array([35, 255, 255])  # yellow upper

    # Create a mask for detecting color objects in the frame
    color_mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    # Find contours in the mask
    contours, _ = cv2.findContours(color_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Filter out small contours that may be noise
        if cv2.contourArea(contour) > 500:
            # Get bounding box for each yellow object
            x, y, w, h = cv2.boundingRect(contour)

            # Draw a green bounding box around the yellow object
            cv2.rectangle(frame_bgr, (x, y), (x+w, y+h), (0, 255, 0), 2)       
        
    frame_bdbox = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    frame_boundbox = frame_bdbox.flatten()
    
    bounding_box = np.array([x, y, x+w, y+h])
    
    # center of the bounding box
    bb_center_x = (bounding_box[0] + bounding_box[2]) / 2
    bb_center_y = (bounding_box[1] + bounding_box[3]) / 2

    # center of the frame
    frame_center_x = width / 2
    frame_center_y = height / 2

    # difference between the centers
    dx = frame_center_x - bb_center_x
    dy = frame_center_y - bb_center_y
    
    if debug_opencv:
        dirname = os.path.dirname(os.path.abspath(__file__))

        # Get the parent directory of the current script's directory
        parent_dir = os.path.dirname(dirname)
        
        filename = os.path.join(parent_dir, 'images/test_hsvframe.png')
        cv2.imwrite(filename, cv2.flip(hsv_frame, -1))

        filename = os.path.join(parent_dir, 'images/test_bgr_filtered.png')
        cv2.imwrite(filename, cv2.flip(frame_bgr, -1))
        
        filename = os.path.join(parent_dir, 'images/test_filtered.png')
        cv2.imwrite(filename, cv2.flip(frame_bdbox, -1))
    
    return frame_boundbox, bounding_box, dx, dy