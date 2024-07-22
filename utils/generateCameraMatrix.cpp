#include <opencv2/opencv.hpp>
#include <mujoco/mujoco.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>


// Function to generate the camera matrix from MuJoCo parameters
cv::Mat generateCameraMatrix(mjModel* model, mjrContext* context, const char* cam_name) {
    
    int H = context->offHeight;
    int W = context->offWidth;

    // set camera
    int cam_id = mj_name2id(model, mjOBJ_CAMERA, cam_name);

    double fovy = model->cam_fovy[cam_id];
    // printf("fovy = %f \n", fovy);

    double aspect_ratio = W / H;
    double fovx =  2 * atan(tan(fovy * M_PI / 360) * aspect_ratio) * 180 / M_PI;
    // printf("fovx = %f \n", fovx);


    double fy = 0.5 * H / tan(fovy * M_PI / 360);
    double fx = 0.5 * W / tan(fovx * M_PI / 360);
    
    // From go1.urdf https://github.com/unitreerobotics/unitree_ros/blob/6a7a9609cebd853062f2182f97dcfbdb711cd61a/robots/go1_description/urdf/go1.urdf#L1348C22-L1348C33
    // double cx = 0.0045;
    // double cy = 0.0039;
    double cx = W / 2;
    double cy = H / 2;

    // Construct the camera matrix
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1
    );
    
    // printf("Fx = %f; Fy = %f; Px = %f; Py = %f \n", cameraMatrix.at<double>(0, 0), cameraMatrix.at<double>(1, 1), cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2));

    return cameraMatrix;

}