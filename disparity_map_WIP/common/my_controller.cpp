// ControlFunction.cpp
#include "my_controller.hpp"
#include "../../utils/boundingbox.hpp"

void init_controller(){
    // initialize controller
}

void mycontroller(const mjModel* m, mjData* d, BoundingBox& result)
{

    // Get the body ID
    int body_id = mj_name2id(m, mjOBJ_BODY, "obstacle1");  // move obstacle 1

    // Check if body exists
    if (body_id == -1) {
        printf("Error: Body 'obstacle1' not found!\n");
        return;
    }

    // Compute index in xfrc_applied array
    int index = body_id * 6;

    // Apply a force in the y-direction of the global frame
    d->xfrc_applied[index + 0] = 1;    // Force along x-axis
    d->xfrc_applied[index + 1] = 0.0;   // Force along y-axis
    d->xfrc_applied[index + 2] = 0.0;   // Force along z-axis
    d->xfrc_applied[index + 3] = 0.0;   // Torque around x-axis (roll)
    d->xfrc_applied[index + 4] = 0.0;   // Torque around y-axis (pitch)
    d->xfrc_applied[index + 5] = 0.0;   // Torque around z-axis (yaw)

    // Control logic: keep target cube in frame
    if (m->nu > 0)  // Ensure control array exists
    {
        double K = 0.1;
        if (result.dx != result.frame_width / 2) {
            d->ctrl[0] = K * result.dx;
        } else {
            d->ctrl[0] = 0;
        }
    }

}