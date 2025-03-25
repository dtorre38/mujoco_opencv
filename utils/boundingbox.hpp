#ifndef BOUNDINGBOX_HPP
#define BOUNDINGBOX_HPP

#include <opencv2/core.hpp>
#include <array>

typedef struct
	{
		float obst_xpos;
		float obst_ypos;
		float obst_zpos;

		// depth for one row: Wx1 = 640x1
		float depthW[640]; // depth in meters
		float depth_cartW[640][2];  // [depth_x_cart, depth_y_cart]
		// float depth_cartW[640][3];  // [depth_x_cart, depth_y_cart, depth_z_cart]

		// depth for one row: FOVx1 = 91x1
		float obstacle_pos[100][5];  // [x, y,z, r, intetionally_left_blank]
		float depth[91]; // depth in meters
		float depth_cart[91][2];  // [depth_x_cart, depth_y_cart]
		// float depth_cart[480][3];  // [depth_x_cart, depth_y_cart, depth_z_cart]

		// FOV depth for one row (min depeth per column): (FOV+1)x1 = 91x1
		// float obstacle_pos[100][5];  // [x, y,z, r, intetionally_left_blank]
		// float depth[91]; // depth in meters
		// float depth_cart[91][2];  // [depth_x_cart, depth_y_cart]
		// float depth_cart[91][3];  // [depth_x_cart, depth_y_cart, depth_z_cart]

		// entire depth image: HxW = 480x640=307200
		// float obstacle_pos[100][5];  // [x, y,z, r, intetionally_left_blank]
		// float depth[307200]; // depth in meters
		// float depth_cart[307200][2];  // [depth_x_cart, depth_y_cart]
		// float depth_cart[307200][2];  // [depth_x_cart, depth_y_cart, depth_z_cart]

	} EnvState;

struct BoundingBox {
    cv::Mat rgbFrame;
    cv::Mat depthFrame;
    cv::Mat depth8Frame;
    std::array<int, 4> boundingBox{};  // [x, y, x+w, y+h]: bottom left and top right coordinates
    double dx = 0.0;  // frame center xpos - bounding box center xpos
    double dy = 0.0;  // frame center ypos - bounding box center ypos
    int frame_width = 0;
    int frame_height = 0;

    double mujoco_distance;
    int id;
    double aruco_distance;
};

#endif // BOUNDINGBOX_HPP
