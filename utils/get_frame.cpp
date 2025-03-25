#include <opencv2/opencv.hpp>
#include <array>
#include <iostream>

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
    unsigned char* depth8 = (unsigned char*)malloc(height * width * 3 * sizeof(unsigned char));
    float* depthm = (float*)malloc(height * width * 1 * sizeof(float));

    mjr_render(offscreen_viewport, scene, context);

    // 4. Read the pixels
    mjr_readPixels(rgb, depth, offscreen_viewport, context);

    // convert to meters
    float extent = model->stat.extent;
    float near = model->vis.map.znear * extent;
    float far = model->vis.map.zfar * extent;
    for (int i=0; i< height * width; ++i) {
    depthm[i] = near / (1.0f - depth[i] * (1.0f - near/far));
    }

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

    // mjr_drawPixels(depth8, NULL, offscreen_viewport, context);

    // Package the data
    frameData.rgb = rgb;
    frameData.depth = depth;
    frameData.depthm = depthm;
    frameData.depth8 = depth8;
    frameData.viewport = offscreen_viewport;
    frameData.width = width;
    frameData.height = height;
}