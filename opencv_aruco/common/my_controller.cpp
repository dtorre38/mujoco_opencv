// ControlFunction.cpp
#include "my_controller.hpp"
#include "../../utils/boundingbox.hpp"

void init_controller(){
    // initialize controller
}

void mycontroller(const mjModel* m, mjData* d, BoundingBox& result)
{
    // Get the body ID
    int body_id = mj_name2id(m, mjOBJ_BODY, "obstacle1");

    // move obstacle 1: aruco cube
    // Apply a force in the y-direction of the global frame
    d->xfrc_applied[6 * body_id + 0] = 0.0;   // Force along x-axis
    d->xfrc_applied[6 * body_id + 1] = 1000;  // Force along y-axis
    d->xfrc_applied[6 * body_id + 2] = 0.0;   // Force along z-axis
    d->xfrc_applied[6 * body_id + 3] = 0.0;   // Torque around x-axis
    d->xfrc_applied[6 * body_id + 4] = 0.0;   // Torque around y-axis
    d->xfrc_applied[6 * body_id + 5] = 0.0;   // Torque around z-axis

    // keep aruco cube in frame
    if (result.dx != result.frame_width/2){
        double K = 0.1;
        d->ctrl[0] = K * result.dx;
    }
    else{
        d->ctrl[0] = 0;
    }

}