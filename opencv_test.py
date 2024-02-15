import mujoco as mj
from mujoco.glfw import glfw
import matplotlib.pyplot as plt
import numpy as np
import os
import cv2
import numpy as np


xml_path = 'box.xml' #xml file (assumes this is in the same folder as this file)
simend = 100 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0


def init_controller(model,data):
    pass


def controller(model, data):
    # desired obstacle to be tracked
    obst_des = 1
    data.qfrc_applied[obst_des] = 0.1


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)


def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)


def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)


def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)


def render_insetscreen(camera_name, loc_x, loc_y, width=640, height=480):
    # Bottom Placement
    # bottom left: loc_x = 0, loc_y = 0
    # bottom middle: loc_x = 0.5*(viewport_width - width), loc_y = 0
    # bottom right: loc_x = viewport_width - width, loc_y = 0
    # Middle Placement
    # middle left: loc_x = 0, loc_y = 0.5*(viewport_width - width)
    # middle: loc_x = 0.5*(viewport_width - width), loc_y = 0.5*(viewport_width - width)
    # middle right: loc_x = viewport_width - width, loc_y = 0.5*(viewport_width - width)
    # Top Placement
    # top left: loc_x = 0, loc_y = viewport_height - height
    # top middle: loc_x = 0.5*(viewport_width - width), loc_y = viewport_height - height
    # top right: loc_x = viewport_width - width, loc_y = viewport_height - height
    
    # Adding an inset window from a different perspective
    # https://github.com/google-deepmind/mujoco/issues/744#issuecomment-1442221178
    # 1. Create a rectangular viewport in the upper right corner for example.
    offscreen_viewport = mj.MjrRect(int(loc_x), int(loc_y), width, height)
    offscreen_context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_100.value)
    
    # 2. Specify a different camera view by updating the scene with mjv_updateScene.
    # Set the camera to the specified view
    camera_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, camera_name)
    offscreen_cam = mj.MjvCamera()
    offscreen_cam.type = mj.mjtCamera.mjCAMERA_FIXED
    offscreen_cam.fixedcamid = camera_id

    # Update scene for the off-screen camera
    mj.mjv_updateScene(model, data, opt, None, offscreen_cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    
    # 3.Render the scene in the offscreen buffer with mjr_render.
    pixels = np.zeros((height*width*3, 1), dtype=np.uint8)  # Placeholder for pixel data
    mj.mjr_render(offscreen_viewport, scene, offscreen_context)
    
    # 4. Read the pixels with mjr_readPixels.
    mj.mjr_readPixels(pixels, None, offscreen_viewport, offscreen_context)
    
    # 5. Call mjr_drawPixels using the rectangular viewport you created in step 1.
    mj.mjr_drawPixels(pixels, None, offscreen_viewport, offscreen_context)


# Assuming `frame` is a captured image from the MuJoCo simulation
def detect_and_draw_bound(camera_name, loc_x, loc_y, width=640, height=480):
    # Bottom Placement
    # bottom left: loc_x = 0, loc_y = 0
    # bottom middle: loc_x = 0.5*(viewport_width - width), loc_y = 0
    # bottom right: loc_x = viewport_width - width, loc_y = 0
    # Middle Placement
    # middle left: loc_x = 0, loc_y = 0.5*(viewport_width - width)
    # middle: loc_x = 0.5*(viewport_width - width), loc_y = 0.5*(viewport_width - width)
    # middle right: loc_x = viewport_width - width, loc_y = 0.5*(viewport_width - width)
    # Top Placement
    # top left: loc_x = 0, loc_y = viewport_height - height
    # top middle: loc_x = 0.5*(viewport_width - width), loc_y = viewport_height - height
    # top right: loc_x = viewport_width - width, loc_y = viewport_height - height
    
    # Adding an inset window from a different perspective
    # https://github.com/google-deepmind/mujoco/issues/744#issuecomment-1442221178
    # 1. Create a rectangular viewport in the upper right corner for example.
    offscreen_viewport = mj.MjrRect(int(loc_x), int(loc_y), width, height)
    offscreen_context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_100.value)
    
    # 2. Specify a different camera view by updating the scene with mjv_updateScene.
    # Set the camera to the specified view
    camera_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, camera_name)
    offscreen_cam = mj.MjvCamera()
    offscreen_cam.type = mj.mjtCamera.mjCAMERA_FIXED
    offscreen_cam.fixedcamid = camera_id

    # Update scene for the off-screen camera
    mj.mjv_updateScene(model, data, opt, None, offscreen_cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    
    # 3.Render the scene in the offscreen buffer with mjr_render.
    frame = np.zeros((height*width*3, 1), dtype=np.uint8)  # Placeholder for pixel data
    mj.mjr_render(offscreen_viewport, scene, offscreen_context)
    
    # 4. Read the pixels with mjr_readPixels.
    mj.mjr_readPixels(frame, None, offscreen_viewport, offscreen_context)
    
    # Reshape mujoco frame to 3D array for opencv input
    frame_reshaped = frame.reshape((height, width, 3))

    # if the image is in RGB format, convert it to BGR.
    frame_bgr = cv2.cvtColor(frame_reshaped, cv2.COLOR_RGB2BGR)
    
    # dirname = os.path.dirname(__file__)
    # filename = os.path.join(dirname, 'test_bgr.png')
    # cv2.imwrite(filename, cv2.flip(frame_bgr, -1))

    # Convert frame to HSV (Hue, Saturation, Value) color space for easier color detection
    hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    
    # dirname = os.path.dirname(__file__)
    # filename = os.path.join(dirname, 'test_hsvframe.png')
    # cv2.imwrite(filename, cv2.flip(hsv_frame, -1))

    # Define range for color in HSV
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
            # print(x, y, w, h)

            # Draw a green bounding box around the yellow object
            cv2.rectangle(frame_bgr, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            dirname = os.path.dirname(__file__)
            # filename = os.path.join(dirname, 'test_rgb_filtered.png')
            # cv2.imwrite(filename, cv2.flip(frame_bgr, -1))

    # Assuming `frame` is the modified frame with bounding boxes, in BGR format, and of shape (height, width, 3)
    # Convert frame from BGR to RGB
    frame_bdbox = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    
    # dirname = os.path.dirname(__file__)
    # filename = os.path.join(dirname, 'test_filtered.png')
    # cv2.imwrite(filename, cv2.flip(frame_bdbox, -1))

    # Flatten the frame to match MuJoCo's expected input format (height*width*3, 1)
    frame_boundbox = frame_bdbox.flatten()
    
    # 5. Call mjr_drawPixels using the rectangular viewport you created in step 1.
    mj.mjr_drawPixels(frame_boundbox, None, offscreen_viewport, offscreen_context)
  

#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname, xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
# cam.azimuth = 90
# cam.elevation = -45
# cam.distance = 2
# cam.lookat = np.array([0.0, 0.0, 0])
cam.azimuth = 90 ; cam.elevation = -1.57 ; cam.distance =  9
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])

#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        mj.mj_step(model, data)

    if (data.time>=simend):
        break

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    
    # set second screen on upper right
    width = 640
    height = 480
    render_insetscreen('robot_camera', loc_x=viewport_width - width, loc_y=viewport_height - height, width=width, height=height)

    detect_and_draw_bound('robot_camera', loc_x=viewport_width - width, loc_y=0, width=640, height=480)
            
    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()
