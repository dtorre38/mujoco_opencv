import mujoco as mj
from mujoco.glfw import glfw
import os

import numpy as np
import matplotlib.pyplot as plt

from utils.detect_and_draw_bound import detect_and_draw_bound
from utils.get_frame import get_frame

xml_path = 'model/box.xml'  # xml file (assumes this is in the same folder as this file)
simend = 100  # simulation time
print_camera_config = 0  # set to 1 to print camera config
                         # this is useful for initializing view of the model

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# inset screen width and height
frame_width = 640
frame_height = 480
dx = 0  # frame center xpos - bounding box center xpos
dy = 0  # frame center ypos - bounding box center ypos
bounding_box = np.array([0, 0, 0, 0])  # [x, y, x+w, y+h]: bottom left and top right coordinates


def init_controller(model, data):
    pass


def controller(model, data):
    # move desired obstacle to be tracked
    obst_des = 1
    data.qfrc_applied[obst_des] = 0.1
    
    # sigmoid for tracking - decent results, better than manually tuned pd controller
    # u = 0.35 / (1 + np.exp(-data.qpos[obst_des]))
    
    # if data.qpos[0] < np.pi/2:
    #     data.ctrl[0] = u
    # else:
    #     data.ctrl[0] = 0
        
    # pd controller to track object
    if dx != frame_width/2:
        K = 0.1
        data.ctrl[0] = K*dx
    else:
        data.ctrl[0] = 0
    
    
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
# window = glfw.create_window(1200, 900, "Demo", None, None)
window = glfw.create_window(900, 600, "OpenCV Tracking", None, None)
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
cam.azimuth = 81 ; cam.elevation = -25 ; cam.distance =  9
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

    # print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    
    # get offscreen simulation frame
    frame, offscreen_viewport = get_frame(model, data, opt, scene, context, 'robot_camera', loc_x=viewport_width - frame_width, loc_y=viewport_height - frame_height, width=frame_width, height=frame_height)
    
    # create bounding box using OpenCV
    frame_boundbox, bounding_box, dx, dy = detect_and_draw_bound(frame, width=frame_width, height=frame_height)
    
    # render bounding box on inset frame
    mj.mjr_drawPixels(frame_boundbox, None, offscreen_viewport, context)
    
    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()
