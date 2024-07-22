#include <iostream>
#include <array>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
// #include <opencv2/objdetect/aruco_detector.hpp>

#include "boundingbox.hpp"
#include "generateCameraMatrix.cpp"

void aruco_tracking(BoundingBox& result, mjModel* model,  mjrContext* context, const char* cam_name) 
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
    // cv::cvtColor(result.rgbFrame, result.rgbFrame, cv::COLOR_BGR2RGB); // bgr to rgb
 
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
    cv::Mat camMatrix = generateCameraMatrix(model, context, cam_name);
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
    bool debug_opencv = false; // Set this to true for debugging
    if (debug_opencv) {
        cv::cvtColor(imageCopy, imageCopy, cv::COLOR_BGR2RGB); // bgr to rgb
        cv::Mat imageCopy_flipped;
        cv::flip(imageCopy, imageCopy_flipped, 0);
        cv::imwrite("aruco.png", imageCopy_flipped);

        cv::cvtColor(result.rgbFrame, result.rgbFrame, cv::COLOR_BGR2RGB); // bgr to rgb
        cv::Mat rgb_flipped;
        cv::flip(result.rgbFrame, rgb_flipped, 0);
        cv::imwrite("rgb.png", rgb_flipped);
        
        cv::Mat depth8_flipped;
        cv::flip(result.depth8Frame, depth8_flipped, 0);
        cv::imwrite("depth8.png", depth8_flipped);
    }
}



void aruco_error(BoundingBox& result, mjModel* model, mjData* data){
    // char* body_name;
    // int bodyid;
    // mjOBJ_BODY,                     // body
    // mjOBJ_XBODY,                    // body, used to access regular frame instead of i-frame

    const char* robot_name = "robot";
    int robot_bodyid = mj_name2id(model, mjOBJ_BODY, robot_name);
    // printf("robot pos (x, y): %f, %f \n", d->xpos[3*robot_bodyid + 0], d->xpos[3*robot_bodyid + 1]);

    const char* body_name = "obstacle1";
    int obstacle1_bodyid = mj_name2id(model, mjOBJ_BODY, body_name);
    // printf("obstacle pos (x, y): %f, %f \n", d->xpos[3*obstacle1_bodyid + 0], d->xpos[3*obstacle1_bodyid + 1]);

    // Get the quaternion representing the rotation of the rotating cube
    const double* quat = &data->cvel[3 * robot_bodyid]; // Assuming 'cvel' contains quaternion velocities, adjust as needed

    // Convert quaternion to rotation matrix
    double rotation_matrix[9]; // Flattened 1D array to hold the rotation matrix
    mju_quat2Mat(rotation_matrix, quat); // Fill the rotation matrix

    // Define the reference point on the robot (+x side face) in robot's local coordinates
    double ref_point_robot_local[3] = {1.0, 0.0, 0.0}; // Assuming the reference point is 1 unit along the x-axis from the origin

    // Transform the reference point from robot's local coordinates to world coordinates
    double ref_point_robot_world[3];
    mju_rotVecMat(ref_point_robot_world, ref_point_robot_local, rotation_matrix); // Rotate the reference point by the rotation matrix

    // Calculate the position of the reference point in world coordinates
    double ref_point_robot_pos[3];
    mju_add3(ref_point_robot_pos, &data->xpos[3 * robot_bodyid], ref_point_robot_world); // Add the rotated reference point to the robot's position

    // Calculate the relative position vector between the reference point on the robot and the center of the translating obstacle
    double dx_world = ref_point_robot_pos[0] - data->xpos[3 * obstacle1_bodyid + 0];
    double dy_world = ref_point_robot_pos[1] - data->xpos[3 * obstacle1_bodyid + 1];

    // Consider cube sizes
    double cube_size_offset = 0.5;
    double x_relative_world = dx_world + cube_size_offset;
    double y_relative_world = dy_world + cube_size_offset;

    // Calculate the distance in world coordinates (Euclidean distance in this example)
    result.mujoco_distance = sqrt(pow(x_relative_world, 2) + pow(y_relative_world, 2));
    double dist_error = abs(result.mujoco_distance - result.aruco_distance);

    printf("Marker ID: %d, MuJoCo Distance: %0.2f meters, Aruco Distance: %0.2f meters, Error: %0.2f \n", result.id, result.mujoco_distance, result.aruco_distance, dist_error);

}
