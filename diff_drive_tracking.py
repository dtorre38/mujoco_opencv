import mujoco as mj
from mujoco.glfw import glfw
import os

import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

from detect_and_draw_bound import detect_and_draw_bound
from get_frame import get_frame


xml_path = 'differential_drive_car.xml' # xml file (assumes this is in the same folder as this file)
simend = 60  # simulation time
print_camera_config = 0  # set to 1 to print camera config
                         # this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# inset screen width and height
frame_width = 320
frame_height = 240
dx = 0  # frame center xpos - bounding box center xpos
dy = 0  # frame center ypos - bounding box center ypos
bounding_box = np.array([0, 0, 0, 0])  # [x, y, x+w, y+h]: bottom left and top right coordinates


def quat2euler(quat_mujoco):
    # mujoco quat is q0,qx,qy,qz
    # scipy quat is qx,qy,qz,q0
    q0 = quat_mujoco[0]
    qx = quat_mujoco[1]
    qy = quat_mujoco[2]
    qz = quat_mujoco[3]
    quat_scipy = np.array([qx,qy,qz,q0])

    r = R.from_quat(quat_scipy)
    euler = r.as_euler('xyz', degrees=False)

    return euler


def euler2quat(euler):
    r = R.from_euler('xyz', euler, degrees=False)
    quat_scipy = r.as_quat()
    # mujoco quat is q0,qx,qy,qz
    # scipy quat is qx,qy,qz,q0
    qx = quat_scipy[0]
    qy = quat_scipy[1]
    qz = quat_scipy[2]
    q0 = quat_scipy[3]
    quat_mujoco = np.array([q0,qx,qy,qz])
    
    return quat_mujoco


def init_controller(model, data):
    # initialize the controller here. This function is called once, in the beginning
    pass


def controller(model, data):
    # put the controller here. This function is called inside the simulation.
    
    # move desired obstacle to be tracked
    nbody = 4
    data.xfrc_applied[nbody][1] = 0.1
    
    # get position of moving obstacle
    # x_ref = data.xpos[nbody][0] 
    # y_ref = data.xpos[nbody][1]
    
    # # get x and y position of the site
    # site_xpos = data.site_xpos[0]
    # x = site_xpos[0]
    # y = site_xpos[1]

    # # get the angle of the car
    # quat = np.array([data.qpos[3],data.qpos[4],data.qpos[5],data.qpos[6]])
    # euler = quat2euler(quat)
    # theta = euler[2]

    # # Following moving object with known COM position of the object
    # # set velocity of left and right wheel
    # Kp = 0.6
    # px = 0.5
    # b = 0.25
    # r = 0.2
    # v = Kp*(np.cos(theta)*(x_ref - x) + np.sin(theta)*(y_ref - y))
    # w = (Kp/px)*(-np.sin(theta)*(x_ref - x) + np.cos(theta)*(y_ref - y))
    # vel_ref_left = v - b*w
    # vel_ref_right = v + b*w
    
    # data.ctrl[0] = vel_ref_left
    # data.ctrl[1] = vel_ref_right
    
    # b = 0.25  # distance between wheels
    # r = 0.2  # radius of wheels
    
    # # tracking moving object via rotation (aiming to keep object centered with help of bounding box)
    # K = 0.1
    # w = -K*dx
    
    # if dx != frame_width/2:
    #     # Calculate wheel velocities for rotating in place
    #     data.ctrl[0] = w * b / (2 * r)
    #     data.ctrl[1] = -w * b / (2 * r)
    # else:
    #     data.ctrl[0] = 0.0
    #     data.ctrl[1] = 0.0
    
    # tracking moving obstacle by following - not working well
    b = 0.25  # distance between wheels
    r = 0.2  # radius of wheels
    
    # Set parameters
    K_rotate = 0.1  # Proportional gain for rotation
    K_forward = 0.01  # Proportional gain for forward/backward motion
    desired_distance = 100  # Desired distance from the object (in pixels)

    # Rotation control
    w = -K_rotate * dx

    # Forward/backward control
    object_distance = frame_height - bounding_box[1]
    error_distance = object_distance - desired_distance
    v = K_forward * error_distance

    # Calculate wheel velocities
    left_wheel_velocity = v + w * b / (2 * r)
    right_wheel_velocity = v - w * b / (2 * r)

    # Update motor control commands
    data.ctrl[0] = left_wheel_velocity
    data.ctrl[1] = right_wheel_velocity

    

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
data = mj.MjData(model)                     # MuJoCo data
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
# top view
# cam.azimuth = 90 ; cam.elevation = -90 ; cam.distance =  25
# cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])

# perspective
cam.azimuth = 90.0 ; cam.elevation = -49 ; cam.distance =  25
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])

# initialize the controller
init_controller(model,data)

# set the controller
mj.set_mjcb_control(controller)

t0 = 0
# x_ref0, y_ref0 = curve(t0)
# data.qpos[0] = x_ref0
# data.qpos[1] = y_ref0

euler_x = 0  # rotation about x-axis
euler_y = 0  # rotation about y-axis

# change euler_z such that it's orientation is at desired initial pose
euler_z = 0 #rotation about z-axis

euler = np.array([euler_x,euler_y,euler_z])
quat = euler2quat(euler)
data.qpos[3] = quat[0]
data.qpos[4] = quat[1]
data.qpos[5] = quat[2]
data.qpos[6] = quat[3]

t = []  # save time
x = []  # save x,y coordinates of site
y = []
xref = []  # save the reference motion
yref = []
error_x = []  # error in tracking
error_y = []

while not glfw.window_should_close(window):
    time_prev = data.time

    while data.time - time_prev < 1.0/60.0:
        mj.mj_step(model, data)

    site_xpos = data.site_xpos[0]

    # t.append(data.time)
    # x.append(site_xpos[0])
    # y.append(site_xpos[1])
    # x_ref,y_ref = curve(data.time)
    # xref.append(x_ref)
    # yref.append(y_ref)
    # error_x.append(site_xpos[0]-x_ref)
    # error_y.append(site_xpos[1]-y_ref)

    # if data.time>=simend:
    #     plt.figure(1)
    #     plt.subplot(2, 1, 1)
    #     plt.plot(t,error_x,'k-')
    #     plt.ylabel("error x")
    #     plt.subplot(2, 1, 2)
    #     plt.plot(t,error_y,'k-')
    #     plt.ylabel("error y")
    #     plt.xlabel("t")
    #     plt.title("plot of error vs time")

    #     plt.figure(2)
    #     plt.plot(x,y)
    #     plt.plot(xref,yref,'r-.')
    #     plt.legend(['meas','ref'])
    #     plt.gca().set_aspect('equal')
    #     plt.title("plot of trajectory in x-y plane")

    #     plt.show(block=False)
    #     plt.pause(20)
    #     plt.close()
    #     break

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if print_camera_config==1:
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    # opt.frame = 6  # check mjtFrame
                     # NONE=0,BODY,GEOM,SITE,CAMERA,LIGHT,WORLD
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    
    # get offscreen simulation frame
    frame, offscreen_viewport, offscreen_context = get_frame(model, data, opt, scene, 'robot_camera', loc_x=viewport_width - frame_width, loc_y=viewport_height - frame_height, width=frame_width, height=frame_height)
    
    # create bounding box using OpenCV
    frame_boundbox, bounding_box, dx, dy = detect_and_draw_bound(frame, width=frame_width, height=frame_height)
    
    # render bounding box on inset frame
    mj.mjr_drawPixels(frame_boundbox, None, offscreen_viewport, offscreen_context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()
