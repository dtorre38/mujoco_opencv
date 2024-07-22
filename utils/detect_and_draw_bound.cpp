#include <opencv2/opencv.hpp>
#include <array>
#include <iostream>

#include "boundingbox.hpp"

bool debug_opencv = false;


void detect_and_draw_bound(BoundingBox& result) {

    cv::Mat rgbFrame = result.rgbFrame;
    cv::Mat depth8Frame = result.depth8Frame;
    int height = result.frame_height;
    int width  = result.frame_width;

    // Convert RGB to BGR - incoming is in bgr, but won't display bounding box if it isn't "converted" to bgr
    cv::Mat bgrFrame;
    cv::cvtColor(rgbFrame, bgrFrame, cv::COLOR_RGB2BGR);

    // Convert BGR to HSV (Hue, Saturation, Value) color space for easier color detection
    cv::Mat hsvFrame;
    cv::cvtColor(bgrFrame, hsvFrame, cv::COLOR_BGR2HSV);

    // Define range for color in HSV (link helps define range of values for colors)
    // https://stackoverflow.com/questions/47483951/how-can-i-define-a-threshold-value-to-detect-only-green-colour-objects-in-an-ima/47483966#47483966
    cv::Scalar lowerColor(20, 0, 0);  // color lower
    cv::Scalar upperColor(30, 255, 255); // color upper
    cv::Mat colorMask;
    cv::inRange(hsvFrame, lowerColor, upperColor, colorMask);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(colorMask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Iterate over contours and draw bounding boxes
    cv::Rect maxRect;
    for (const auto& contour : contours) {
        if (cv::contourArea(contour) > 500) { // Filter small contours that may be noise
            cv::Rect rect = cv::boundingRect(contour);
            if (rect.area() > maxRect.area()) {
                maxRect = rect;

                // Draw the bounding box on the original frame
                cv::rectangle(rgbFrame, maxRect, cv::Scalar(0, 255, 0), 2);
            }
        }
    }
    
    // cv::cvtColor(rgbFrame, rgbFrame, cv::COLOR_BGR2RGB);
    result.rgbFrame = rgbFrame;

    result.boundingBox = {maxRect.x, maxRect.y, maxRect.x + maxRect.width, maxRect.y + maxRect.height};
    
    // Calculate center of bounding box and frame
    result.dx = (width - (result.boundingBox[0] + result.boundingBox[2])) / 2.0;
    result.dy = (height - (result.boundingBox[1] + result.boundingBox[3])) / 2.0;

    // Debugging: Save images for verification
    if (debug_opencv) {
        cv::Mat rgb_flipped;
        cv::flip(rgbFrame, rgb_flipped, -1);
        cv::imwrite("images/frame_rgb.png", rgb_flipped);

        cv::Mat bgr_flipped;
        cv::flip(bgrFrame, bgr_flipped, -1);
        cv::imwrite("images/frame_bgr.png", bgr_flipped);

        cv::Mat hsv_flipped;
        cv::flip(hsvFrame, hsv_flipped, -1);
        cv::imwrite("images/frame_hsv.png", hsv_flipped);

        cv::Mat result_flipped;
        cv::flip(result.rgbFrame, result_flipped, -1);
        cv::imwrite("images/frame_boundbox.png", result_flipped);
    }
}
