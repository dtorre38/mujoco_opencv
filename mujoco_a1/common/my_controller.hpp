// my_controller.hpp
#ifndef MY_CONTROLER_HPP
#define MY_CONTROLER_HPP

#include <mujoco/mujoco.h>

void init_controller(mjModel* m, mjData* d);

void my_controller(mjModel* m, mjData* d);

#endif // CONTROLFUNCTION_HPP
