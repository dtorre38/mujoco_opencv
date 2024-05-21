// ControlFunction.cpp
#include "my_controller.hpp"
#include "../../utils/boundingbox.hpp"

void init_controller(){
    // initialize controller
}

void mycontroller(const mjModel* m, mjData* d, BoundingBox& result)
{
    // move obstacle 1: yellow cube
    int nbody = 4;
    d->xfrc_applied[nbody*6 + 1] = 0.1;

    // tracking moving object via rotation (aiming to keep object centered with help of bounding box)
    // double b = 0.25;  // distance between wheels
    // double r = 0.2;  // radius of wheels
    
    // double K = 0.1;
    // double w = -K * result.dx;
    
    // if (result.dx != result.frame_width/2){
    //     // Calculate wheel velocities for rotating in place
    //     d->ctrl[0] = w * b / (2 * r);
    //     d->ctrl[1] = -w * b / (2 * r);
    // }
    // else{
    //     d->ctrl[0] = 0.0;
    //     d->ctrl[1] = 0.0;
    // }

    // tracking moving obstacle by following - sticking too close
    double b = 0.25;  // distance between wheels
    double r = 0.2;  // radius of wheels
    
    // Set parameters
    double K_rotate = 0.1;  // Proportional gain for rotation
    double K_forward = 0.01;  // Proportional gain for forward/backward motion
    double desired_distance = 100;  // Desired distance from the object (in pixels)

    // Rotation control
    double w = -K_rotate * result.dx;

    // Forward/backward control
    double object_distance = result.frame_height - result.boundingBox[1];
    double error_distance = object_distance - desired_distance;
    double v = K_forward * error_distance;

    // Calculate wheel velocities
    double left_wheel_velocity = v + w * b / (2 * r);
    double right_wheel_velocity = v - w * b / (2 * r);
    
    if (result.dx != result.frame_width/2){
        // Update motor control commands
        d->ctrl[0] = left_wheel_velocity;
        d->ctrl[1] = right_wheel_velocity;
    }
    else{
        d->ctrl[0] = 0.0;
        d->ctrl[1] = 0.0;
    }

}