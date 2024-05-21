#include <mujoco/mujoco.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring> // For memcpy

#include "framedata.hpp"


void get_frame(mjModel* model, mjData* data, mjvOption* opt, mjvScene* scene, mjrContext* context, mjvCamera* cam, 
               FrameData& frameData, int loc_x, int loc_y, int width, int height) {
    // 1. Create a rectangular viewport
    mjrRect offscreen_viewport = {loc_x, loc_y, width, height};

    // 2. Update scene for off-screen camera
    mjv_updateScene(model, data, opt, NULL, cam, mjCAT_ALL, scene);

    // 3. Render the scene in the offscreen buffer
    unsigned char* rgb = (unsigned char*)malloc(height * width * 3 * sizeof(unsigned char));
    float* depth = (float*)malloc(height * width * 1 * sizeof(float));
    float* depthm = (float*)malloc(height * width * 1 * sizeof(float));

    mjr_render(offscreen_viewport, scene, context);

    // 4. Read the pixels
    mjr_readPixels(rgb, depth, offscreen_viewport, context);

    // Allocate memory for the left and right images
    // int half_width = width/2;
    // unsigned char* left_rgb = (unsigned char*)malloc(height * half_width * 3 * sizeof(unsigned char));
    // unsigned char* right_rgb = (unsigned char*)malloc(height * half_width * 3 * sizeof(unsigned char));
    // float* left_depth = (float*)malloc(height * half_width * sizeof(float));
    // float* right_depth = (float*)malloc(height * half_width * sizeof(float));

    // // Iterate over each row
    // for (int row = 0; row < height; ++row) {
    //     // Copy left RGB image
    //     memcpy(&left_rgb[row * half_width * 3], 
    //            &rgb[row * width * 3], 
    //            half_width * 3 * sizeof(unsigned char));
        
    //     // Copy right RGB image
    //     memcpy(&right_rgb[row * half_width * 3], 
    //            &rgb[row * width * 3 + half_width * 3], 
    //            half_width * 3 * sizeof(unsigned char));
        
    //     // Copy left depth image
    //     memcpy(&left_depth[row * half_width], 
    //            &depth[row * width], 
    //            half_width * sizeof(float));
        
    //     // Copy right depth image
    //     memcpy(&right_depth[row * half_width], 
    //            &depth[row * width + half_width], 
    //            half_width * sizeof(float));
    // }

    // cv::Mat left(half_width, height, CV_8UC3, left_rgb);
    // cv::cvtColor(left, left, cv::COLOR_BGR2RGB);
    // cv::Mat left_flipped;
    // cv::flip(left, left_flipped, 0);
    // cv::imwrite("left.png", left_flipped);
    
    // cv::Mat right(half_width, height, CV_8UC3, right_rgb);
    // cv::cvtColor(right, right, cv::COLOR_BGR2RGB);
    // cv::Mat right_flipped;
    // cv::flip(right, right_flipped, 0);
    // cv::imwrite("right.png", right_flipped);
    
    // convert to meters
    float extent = model->stat.extent;
    float near = model->vis.map.znear * extent;
    float far = model->vis.map.zfar * extent;
    for (int i=0; i< height * width; ++i) {
        depthm[i] = near / (1.0f - depth[i] * (1.0f - near/far));
    }

    // Convert your 1D array to a 2D cv::Mat
    cv::Mat depth_2d(height, width, CV_32F, depth);

    // Normalize the depth array to fit within the 8-bit range (0-255)
    cv::Mat normalized_depth;
    cv::normalize(depth_2d, normalized_depth, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    // Flip the image vertically and horizontally
    cv::Mat depth_image_flipped;
    cv::flip(normalized_depth, depth_image_flipped, 0); // -1 indicates flipping both axes

    // Save the image
    // cv::imwrite("get_frame_depth.jpg", depth_image_flipped);

    // convert to a 3-channel 8-bit image
    unsigned char* depth8 = (unsigned char*)malloc(height * width * 3 * sizeof(unsigned char));
    for (int i=0; i< height * width; i++) {
        depth8[3*i] = depth8[3*i+1] = depth8[3*i+2] = depth[i];
    }

    // Package the data
    frameData.rgb = rgb;
    frameData.depth = depth;
    frameData.depthm = depthm;
    frameData.depth8 = depth8;
    frameData.viewport = offscreen_viewport;
    frameData.width = width;
    frameData.height = height;
}