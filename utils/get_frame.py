import mujoco as mj
import numpy as np


def get_frame(model, data, opt, scene, context, camera_name, loc_x, loc_y, width=640, height=480):
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
    
    height = int(height)
    width = int(width)
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
    
    # 3.Render the scene in the offscreen buffer with mjr_render.
    frame = np.zeros((height*width*3, 1), dtype=np.uint8)  # Placeholder for pixel data
    mj.mjr_render(offscreen_viewport, scene, context)
    
    # 4. Read the pixels with mjr_readPixels.
    mj.mjr_readPixels(frame, None, offscreen_viewport, context)
    
    # 5. Call mjr_drawPixels using the rectangular viewport you created in step 1.
    # glClear(GL_DEPTH_BUFFER_BIT)  # allows rendering over geometries
    # mj.mjr_drawPixels(frame, None, offscreen_viewport, context)
    
    return frame, offscreen_viewport