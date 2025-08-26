#include "ros/ros.h"
#include "miniarm_vel/MoveArm.h"
#include "miniarm_vel/Manipulator.h"
#include <iostream>

Manipulator* g_arm = nullptr;

bool moveCallback(miniarm_vel::MoveArm::Request &req,
                  miniarm_vel::MoveArm::Response &res)
{
    ROS_INFO("Received target: x=%.2f, y=%.2f, z=%.2f", req.x, req.y, req.z);

    bool reached = false;
    int max_steps = 5000;

    for (int i = 0; ros::ok() && i < max_steps; ++i) {
        reached = g_arm->stepToward(0, 0, 0, req.x, req.y, req.z);
        ros::Duration(0.01).sleep(); // 100 Hz
        if (reached) break;
    }

    if (reached) {
        res.success = true;
        res.message = "Target reached.";
        ROS_INFO("Target reached.");
    } else {
        res.success = false;
        res.message = "Failed to reach target within step limit.";
        ROS_WARN("Failed to reach target.");
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_server");
    ros::NodeHandle nh;

    Manipulator arm;
    g_arm = &arm;

    ros::ServiceServer service = nh.advertiseService("move_arm", moveCallback);

    ROS_INFO("Arm server ready. Waiting for client requests...");
    ros::spin();

    return 0;
}
