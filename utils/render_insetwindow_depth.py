import mujoco as mj
import numpy as np

import cv2

def render_insetscreen_depth(model, data, opt, scene, context, camera_name, loc_x, loc_y, width=640, height=480):
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
    
    rgb = np.zeros((height * width * 3, 1), dtype=np.uint8)  # Placeholder for rgb data
    
    # depth: [numpy.ndarray[numpy.float32[m, 1]]], values are in range [0, 1]
    depth = np.zeros((height * width, 1), dtype=np.float32)
    depth8 = np.zeros((height, width, 3), dtype=np.uint8) 

    # 4. Read the pixels with mjr_readPixels.
    mj.mjr_readPixels(rgb, depth, offscreen_viewport, context)
    
    # Convert to meters - Does not work for rendering
    # extent = model.stat.extent
    # near = model.vis.map.znear * extent
    # far = model.vis.map.zfar * extent
    # for i in range(height*width):
    #     depth[i] = near / (1.0 - depth[i] * (1.0 - near / far))
    
    # Reshape the flat depth array to its original 2D shape
    depth_2d = depth.reshape(height, width)
    
    # Normalize the depth array to fit within the 8-bit range (0-255)
    normalized_depth = cv2.normalize(depth_2d, None, 0, 255, cv2.NORM_MINMAX)
    
    depth_image_8bit = cv2.flip(normalized_depth, 0)
    cv2.imwrite('images/depth_renderinstetwindowdepth.jpg', depth_image_8bit)

    # Fill each channel of depth8 with depth_image_8bit to maintain grayscale
    for i in range(3): 
        depth8[:, :, i] = normalized_depth

    # mjr_drawPixels expects data in a flat array format
    depth8 = depth8.reshape(-1)

    # Convert to a 3-channel 8-bit image
    # depth8 = np.zeros((height * width * 3), dtype=np.uint8)
    # for i in range(width * height):
    #     depth8[3*i] = depth8[3*i+1] = depth8[3*i+2] = depth[i] * 255

    # 5. Call mjr_drawPixels using the rectangular viewport you created in step 1.
    mj.mjr_drawPixels(depth8, None, offscreen_viewport, context)
