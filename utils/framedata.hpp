#ifndef FRAMEDATA_HPP
#define FRAMEDATA_HPP

#include <mujoco/mujoco.h>
#include <cstdlib>

struct FrameData {
    unsigned char* rgb;
    float* depth;
    float* depthm;
    unsigned char* depth8;
    mjrRect viewport;
    int FOV;
    int width;
    int height;

    // envstate
    float obstacle_pos[100][5];  // [x, y, z, r, intetionally_left_blank]
    float envstate_depthm[91]; // depth in meters
    float depth_cart[91][2];  // [depth_x_cart, depth_y_cart]

    // camera intrinsics
    double fovy;
    double fovx;

    double fx;
    double fy;
    double cx;
    double cy;

    cv::Mat rgbFrame;

    cv::Mat depthm8Frame;
    
    cv::Mat depthFrame;
    cv::Mat depth8Frame;
    cv::Mat boundingboxdepth;

    cv::Mat udepthmap;
    cv::Mat udepthmap8;
    cv::Mat binaryudepthmap8;
    cv::Mat boundingboxudepth8;

    cv::Mat vdepthmap;
    cv::Mat vdepthmap8;
    cv::Mat binaryvdepthmap8;
    cv::Mat boundingboxvdepth8;

    double mujoco_distance;
    int id;
    double aruco_distance;

    FrameData(int w, int h) : width(w), height(h), rgb(static_cast<unsigned char*>(malloc(h * w * 3))), depth((float*)(malloc(h * w * 1))), depthm((float*)(malloc(h * w * 1))), depth8(static_cast<unsigned char*>(malloc(h * w * 3))) {}
    
    ~FrameData() {
        free(rgb);
        free(depth);
        free(depthm);
        free(depth8);
    }

    // Prevent copying
    FrameData(const FrameData&) = delete;
    FrameData& operator=(const FrameData&) = delete;
};

#endif // FRAMEDATA_HPP