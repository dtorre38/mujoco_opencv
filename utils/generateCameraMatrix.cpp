#include <opencv2/opencv.hpp>
#include <mujoco/mujoco.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>


// Function to generate the camera matrix from MuJoCo parameters
cv::Mat generateCameraMatrix(mjModel* model, int frame_width, int frame_height) {

    // set robot perspective as the main camera
    const char* robot_cam = "robot_camera";
    int cam_id = mj_name2id(model, mjOBJ_CAMERA, robot_cam);

    double fovy = model->cam_fovy[cam_id];
    // printf("fovy = %f \n", fovy);

    double aspect_ratio = frame_width/frame_height;
    double fovx =  2 * atan(tan(fovy * M_PI / 360) * aspect_ratio) * 180 / M_PI;
    // printf("fovx = %f \n", fovx);


    double fy = 0.5 * frame_height / tan(fovy * M_PI / 360);
    double fx = 0.5 * frame_width / tan(fovx * M_PI / 360);
    
    // From go1.urdf https://github.com/unitreerobotics/unitree_ros/blob/6a7a9609cebd853062f2182f97dcfbdb711cd61a/robots/go1_description/urdf/go1.urdf#L1348C22-L1348C33
    // double cx = 0.0045;
    // double cy = 0.0039;
    double cx = frame_width / 2;
    double cy = frame_height / 2;

    // Construct the camera matrix
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1
    );

    return cameraMatrix;

}