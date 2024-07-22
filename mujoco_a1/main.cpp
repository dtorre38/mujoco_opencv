#include <iostream>
#include <string>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <opencv2/opencv.hpp>
#include <cstdint>

#include "../utils/render_insetwindow.c"
#include "../utils/get_frame.cpp"
#include "../utils/detect_and_draw_bound.cpp"
#include "../utils/aruco_tracking.cpp"
#include "../utils/framedata.hpp"
#include "../utils/boundingbox.hpp"

#include "include/StereoCameraCommon2.hpp"

#include "common/my_controller.cpp"

#define dtime 0.001

// Global variables
mjModel* m = nullptr; // MuJoCo model
mjData* d = nullptr; // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

bool print_camera_config = 0;  // set to 1 to print camera config
                               // this is useful for initializing view of the model)

// controller related variables
double t_control = 0;
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

// inset screen width and height
int frame_width = 0.5 * 640;
int frame_height = 0.5 * 480;

DepthCamera depthCamera(frame_width, frame_height);


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// Main function
int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " modelfile" << std::endl;
        return 0;
    }

    // Initialize MuJoCo and load model
    std::string modelFile = argv[1];
    char error[1000] = "Could not load binary model";
    if (modelFile.size() > 4 && modelFile.substr(modelFile.size() - 4) == ".mjb") {
        m = mj_loadModel(modelFile.c_str(), nullptr);
    } else {
        m = mj_loadXML(modelFile.c_str(), nullptr, error, 1000);
    }
    if (!m) {
        throw std::runtime_error("Load model error: " + std::string(error));
    }

    // make data
    d = mj_makeData(m);

    // init GLFW
    if (!glfwInit()) {
        throw std::runtime_error("Could not initialize GLFW");
    }

    // create window, make OpenGL context current, request v-sync
    int viewport_width = 900;
    int viewport_height = 600;
    GLFWwindow* window = glfwCreateWindow(viewport_width, viewport_height, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {90, -30, 10, 0.0, 0.0, 1.25};
     cam.azimuth = arr_view[0];
     cam.elevation = arr_view[1];
     cam.distance = arr_view[2];
     cam.lookat[0] = arr_view[3];
     cam.lookat[1] = arr_view[4];
     cam.lookat[2] = arr_view[5];

    //  d->qpos[0]=1.57; // pi/2

    // mjcb_control = mycontroller(m, d, result);

    // initialize robot perspective camera
    mjvCamera robot_cam;
    const char* robot_front_cam = "robot_camera";
    int rcam_id = mj_name2id(m, mjOBJ_CAMERA, robot_front_cam);
    robot_cam.type = mjCAMERA_FIXED;
    robot_cam.fixedcamid = rcam_id;

    // create BoundingBox struct
    BoundingBox result;
    result.frame_width = frame_width;
    result.frame_height = frame_height;

    // Set up OpenCV window
    cv::namedWindow("Depth View", cv::WINDOW_AUTOSIZE);

    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            // mj_step(m, d);
            // mycontroller(m, d, result);

            mj_step(m, d);

            if (d->time - t_control > dtime-1e-5)
            {
                mycontroller(m,d,result);
                t_control = d->time;
            }
        }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        if (print_camera_config){
            printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);
        }

        int loc_x = viewport.width - frame_width;
        int loc_y = viewport.height - frame_height;
        // render_insetscreen(m, d, &opt, &scn, &con, "robot_camera", loc_x, loc_y, frame_width, frame_height);
        
        FrameData frameData(frame_width, frame_height);
        get_frame(m, d, &opt, &scn, &con, &robot_cam, frameData, loc_x, loc_y, frame_width, frame_height);

        // Reshape mujoco frame [height x width, 1] to 3D array for opencv input [height, width, 3]
        cv::Mat frame_bgr = cv::Mat(frame_height, frame_width, CV_8UC3, frameData.rgb); // maps to bgr image
        cv::Mat frame_depth = cv::Mat(frame_height, frame_width, CV_32F, frameData.depth);
        cv::Mat frame_depth8 = cv::Mat(frame_height, frame_width, CV_8UC3, frameData.depth8);

        result.rgbFrame = frame_bgr;
        result.depthFrame = frame_depth;
        result.depth8Frame = frame_depth8;

        // aruco tracking
        // aruco_tracking(result);

        // Draw bounding box on frame
        // detect_and_draw_bound(result);
        
        if (!result.rgbFrame.empty()) {
            cv::Mat rgbFrame;
            cv::Mat depthFrame;

            // Ensure data is contiguous before using it
            if (!rgbFrame.isContinuous()) {
                rgbFrame = result.rgbFrame;
                depthFrame = result.depthFrame;
                glClear(GL_DEPTH_BUFFER_BIT);
                // mjr_drawPixels(rgbFrame.data, NULL, frameData.viewport, &con);
                mjr_drawPixels(frame_depth8.data, NULL, frameData.viewport, &con);
            }
        }

        // Display depth image
        cv::Mat depth;
        cv::flip(frame_depth8.clone(), depth, 0);
        cv::imshow("Depth View", depth);
        // Check for exit key
        if (cv::waitKey(1) == 27) break;

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    // mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}
