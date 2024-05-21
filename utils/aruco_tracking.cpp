#include <iostream>
#include <array>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
// #include <opencv2/objdetect/aruco_detector.hpp>

#include "boundingbox.hpp"
#include "generateCameraMatrix.cpp"

void aruco_tracking(BoundingBox& result, mjModel* model) 
{
    int height = result.frame_height;
    int width  = result.frame_width;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000); // change dictionary

    // Adjust detector parameters for better detection
    // detectorParams->adaptiveThreshWinSizeMin = 3;
    // detectorParams->adaptiveThreshWinSizeMax = 23;
    // detectorParams->adaptiveThreshWinSizeStep = 10;
    // detectorParams->minMarkerPerimeterRate = 0.04;
    // detectorParams->maxMarkerPerimeterRate = 4.0;
    // detectorParams->polygonalApproxAccuracyRate = 0.05;
    // detectorParams->minCornerDistanceRate = 0.05;
    // detectorParams->minDistanceToBorder = 3;
    // detectorParams->minMarkerDistanceRate = 0.1;


    double markerLength = 0.57449; // 0.574898785425101 meters
    
    // Set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

    cv::Mat imageCopy;

    // cv::Mat gray;
    // cv::cvtColor(result.rgbFrame, gray, cv::COLOR_BGR2GRAY); // grayscale for easier detection
    cv::cvtColor(result.rgbFrame, result.rgbFrame, cv::COLOR_BGR2RGB); // bgr to rgb
 
    // detect markers and estimate pose
    // cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
    cv::aruco::detectMarkers(result.rgbFrame, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

    // cv::Mat outputFrame = gray.clone();
    cv::Mat outputFrame = result.rgbFrame.clone();
    cv::aruco::drawDetectedMarkers(outputFrame, markerCorners, markerIds);

    size_t nMarkers = markerCorners.size();
    std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

    bool estimatePose = true; // Set this to true if you want to estimate pose
    // cv::Mat camMatrix = (cv::Mat_<double>(3, 3) << 1, 0, result.rgbFrame.cols / 2.0, 0, 1, result.rgbFrame.rows / 2.0, 0, 0, 1);
    cv::Mat camMatrix = generateCameraMatrix(model, width, height);
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    if (estimatePose && !markerIds.empty()) {
        // Calculate pose for each marker
        for (size_t i = 0; i < nMarkers; i++) {
            cv::solvePnP(objPoints, markerCorners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i), false, cv::SOLVEPNP_IPPE_SQUARE);
            
            // Calculate Euclidean distance
            double distance = cv::norm(tvecs[i]);
            
            // Save marker information in the struct
            // result.push_back({ markerIds[i], distance });
            result.id = markerIds[i];
            result.aruco_distance = distance;
            // std::cout << "Marker ID: " << markerIds[i] << ", Distance: " << distance << " meters" << std::endl;
        }
    }

    // Draw results
    // gray.copyTo(imageCopy);
    result.rgbFrame.copyTo(imageCopy);
    if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);

        if (estimatePose) {
            for (unsigned int i = 0; i < markerIds.size(); i++)
                cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
        }
    }

    bool showRejected = false; // Set this to true if you want to show rejected markers
    if (showRejected && !rejectedCandidates.empty())
        cv::aruco::drawDetectedMarkers(imageCopy, rejectedCandidates, cv::noArray(), cv::Scalar(100, 0, 255));

    result.rgbFrame = imageCopy;
    cv::normalize(result.depthFrame, result.depth8Frame, 0, 255, cv::NORM_MINMAX, CV_8U);

    if (!markerCorners.empty()) {
        // Calculate bounding box
        cv::Rect boundingBox = cv::boundingRect(markerCorners[0]);
        for (size_t i = 1; i < markerCorners.size(); i++) {
            boundingBox |= cv::boundingRect(markerCorners[i]);
        }

        result.boundingBox = {boundingBox.x, boundingBox.y, boundingBox.x + boundingBox.width, boundingBox.y + boundingBox.height};

        // Calculate center of bounding box and frame
        result.dx = (width - (result.boundingBox[0] + result.boundingBox[2])) / 2.0;
        result.dy = (height - (result.boundingBox[1] + result.boundingBox[3])) / 2.0;
    }    

    // Debugging: Save images for verification
    bool debug_opencv = true; // Set this to true for debugging
    if (debug_opencv) {
        cv::Mat imageCopy_flipped;
        cv::flip(imageCopy, imageCopy_flipped, 0);
        cv::imwrite("aruco.png", imageCopy_flipped);

        cv::Mat rgb_flipped;
        cv::flip(result.rgbFrame, rgb_flipped, 0);
        cv::imwrite("rgb.png", rgb_flipped);
        
        cv::Mat depth8_flipped;
        cv::flip(result.depth8Frame, depth8_flipped, 0);
        cv::imwrite("depth8.png", depth8_flipped);
    }
}
