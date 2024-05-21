#ifndef BOUNDINGBOX_HPP
#define BOUNDINGBOX_HPP

#include <opencv2/core.hpp>
#include <array>

struct BoundingBox {
    cv::Mat rgbFrame;
    cv::Mat depthFrame;
    cv::Mat depth8Frame;
    std::array<int, 4> boundingBox{};  // [x, y, x+w, y+h]: bottom left and top right coordinates
    double dx = 0.0;  // frame center xpos - bounding box center xpos
    double dy = 0.0;  // frame center ypos - bounding box center ypos
    int frame_width = 0;
    int frame_height = 0;

    double mujoco_distance;
    int id;
    double aruco_distance;
};

#endif // BOUNDINGBOX_HPP
