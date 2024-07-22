#include <opencv2/opencv.hpp>
#include <array>
#include <iostream>


void render_insetscreen(mjModel* model, mjData* data, mjvOption* opt, mjvScene* scene, mjrContext* context,
                        const char* camera_name, int loc_x, int loc_y,
                        int width, int height,
                        int depth_flag) 
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
    mjr_render(offscreen_viewport, scene, context);

    if (!depth_flag){
        unsigned char* rgb = (unsigned char*)malloc(height * width * 3 * sizeof(unsigned char));

        // 4. Read the pixels with mjr_readPixels
        mjr_readPixels(rgb, NULL, offscreen_viewport, context);

        // 5. Draw the pixels with mjr_drawPixels using the rectangular viewport
        // glClear(GL_DEPTH_BUFFER_BIT); // allows bounding box to render over geometries
        mjr_drawPixels(rgb, NULL, offscreen_viewport, context);

        // Free memory
        free(rgb);
    }
    else{
        unsigned char* rgb = (unsigned char *)malloc(height * width * 3 * sizeof(unsigned char));
        float* depth = (float *)malloc(height * width * 1 * sizeof(float));  // read depth buffer, values are in range [0, 1]
        unsigned char* depth8 = (unsigned char *)malloc(height * width * 3 * sizeof(unsigned char));

        // 4. Read the rgb with mjr_readPixels
        mjr_readPixels(NULL, depth, offscreen_viewport, context);

        // Normalize depth to fit within 8-bit range
        float min_depth = FLT_MAX;
        float max_depth = -FLT_MAX;
        for (int i = 0; i < height * width; i++) {
            if (depth[i] < min_depth) min_depth = depth[i];
            if (depth[i] > max_depth) max_depth = depth[i];
        }

        float range = max_depth - min_depth;
        for (int i = 0; i < height * width; i++) {
            depth8[3 * i] = depth8[3 * i + 1] = depth8[3 * i + 2] = (unsigned char)((depth[i] - min_depth) / range * 255.0f);
        }
        
        glClear(GL_DEPTH_BUFFER_BIT);
        mjr_drawPixels(depth8, NULL, offscreen_viewport, context);

        // save image to confirm depth
        cv::Mat depth_image(height, width, CV_8UC3, depth8);
        cv::flip(depth_image, depth_image, 0);
        cv::imwrite("../images/renderinstetwindowdepth.jpg", depth_image);

        // Free allocated memory
        free(rgb);
        free(depth);
        free(depth8);
    }
    
}