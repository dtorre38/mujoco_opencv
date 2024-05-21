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
    int width;
    int height;

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