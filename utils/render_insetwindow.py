import mujoco as mj
import numpy as np
from OpenGL.GL import *
import cv2


def render_insetscreen(model, data, opt, scene, context, camera_name, loc_x, loc_y, width=640, height=480, depth_flag=0):
    """
    Renders an inset view of the simulation scene from a different camera perspective.

    Parameters:
        model (mj.Model): The MuJoCo model.
        data (mj.Data): The MuJoCo simulation data.
        opt (mj.MjOption): The MuJoCo options.
        scene (mj.MjvScene): The MuJoCo scene.
        context (mj.MjrContext): The MuJoCo rendering context.
        camera_name (str): The name of the camera to use for the inset view.
        loc_x (int): The x-coordinate for the location of the inset view.
        loc_y (int): The y-coordinate for the location of the inset view.
        width (int): The width of the inset view.
        height (int): The height of the inset view.
        depth_flag (int): Flag to determine if depth information is rendered (0 for RGB, non-zero for depth).
    """
    
    height = int(height)
    width = int(width)
    
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
    offscreen_viewport = mj.MjrRect(int(loc_x), int(loc_y), int(width), int(height))

    # 2. Specify a different camera view by updating the scene with mjv_updateScene.
    # Set the camera to the specified view
    camera_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, camera_name)
    offscreen_cam = mj.MjvCamera()
    offscreen_cam.type = mj.mjtCamera.mjCAMERA_FIXED
    offscreen_cam.fixedcamid = camera_id

    # Update scene for the off-screen camera
    mj.mjv_updateScene(model, data, opt, None, offscreen_cam, mj.mjtCatBit.mjCAT_ALL.value, scene)

    # 3. Render the scene in the offscreen buffer with mjr_render.
    mj.mjr_render(offscreen_viewport, scene, context)
    
    if not depth_flag:
        # buffer for rgb data
        rgb = np.zeros((height * width * 3, 1), dtype=np.uint8)
        
        # 4. Read the pixels with mjr_readPixels.
        mj.mjr_readPixels(rgb, None, offscreen_viewport, context)

        # 5. Call mjr_drawPixels using the rectangular viewport you created in step 1.
        mj.mjr_drawPixels(rgb, None, offscreen_viewport, context)
    else:
        # buffer for depth data
        # depth: [numpy.ndarray[numpy.float32[m, 1]]], values are in range [0, 1]
        depth = np.zeros((height * width, 1), dtype=np.float32)
        depth8 = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 4. Read the pixels with mjr_readPixels.
        mj.mjr_readPixels(None, depth, offscreen_viewport, context)

        # Shift nearest values to the origin.
        # depth -= depth.min()
        # Scale by 2 mean distances of near rays.
        # depth /= 2*depth[depth <= 1].mean()
        # Scale to [0, 255]
        # depth = 255*np.clip(depth, 0, 1)
        
        # Convert to meters
        # extent = model.stat.extent
        # near = model.vis.map.znear * extent
        # far = model.vis.map.zfar * extent
        # depthm = np.zeros((height * width, 1), dtype=np.float32)
        # for i in range(height*width):
        #     depthm[i] = near / (1.0 - depth[i] * (1.0 - near / far))
        
        # Normalize the depth array to fit within the 8-bit range (0-255)
        depth_2d = depth.reshape(height, width)
        normalized_depth = cv2.normalize(depth_2d, None, 0, 255, cv2.NORM_MINMAX)
        depth_image_8bit = cv2.flip(normalized_depth, 0)
        cv2.imwrite('images/renderinstetwindowdepth.jpg', depth_image_8bit)

        # Convert normalized depth to 3-channel grayscale image
        depth8[:, :, 0] = depth8[:, :, 1] = depth8[:, :, 2] = normalized_depth
        
        # Flatten depth8 for drawing
        depth8 = depth8.reshape(-1)
           
        # 5. Call mjr_drawPixels using the rectangular viewport you created in step 1.
        glClear(GL_DEPTH_BUFFER_BIT)  # allows rendering over geometries
        mj.mjr_drawPixels(depth8, None, offscreen_viewport, context)
