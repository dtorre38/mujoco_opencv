#include <mujoco/mujoco.h>
#include <stdio.h>
#include <stdlib.h>


void render_insetscreen(mjModel* model, mjData* data, mjvOption* opt, mjvScene* scene, mjrContext* context,
                        const char* camera_name, int loc_x, int loc_y,
                        int width, int height) 
{
    // 1. Create a rectangular viewport
    mjrRect offscreen_viewport = {loc_x, loc_y, width, height};

    // 2. Specify a different camera view by updating the scene with mjv_updateScene
    // Set the camera to the specified view
    int camera_id = mj_name2id(model, mjOBJ_CAMERA, camera_name);
    if (camera_id < 0)
    {
        printf("Error: Camera not found\n");
        return;
    }
    mjvCamera offscreen_cam;
    offscreen_cam.type = mjCAMERA_FIXED;
    offscreen_cam.fixedcamid = camera_id;

    // Update scene for off-screen camera
    mjv_updateScene(model, data, opt, NULL, &offscreen_cam, mjCAT_ALL, scene);

    // 3. Render the scene in the offscreen buffer mjr_render
    unsigned char* rgb = (unsigned char*)malloc(height * width * 3 * sizeof(unsigned char));
    mjr_render(offscreen_viewport, scene, context);

    // 4. Read the pixels with mjr_readPixels
    mjr_readPixels(rgb, NULL, offscreen_viewport, context);

    // 5. Draw the pixels with mjr_drawPixels using the rectangular viewport
    mjr_drawPixels(rgb, NULL, offscreen_viewport, context);

    // Free memory
    free(rgb);
}