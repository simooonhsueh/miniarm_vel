#include "ros/ros.h"
#include "miniarm_vel/MoveArm.h"
#include "miniarm_vel/Manipulator.h"
#include "miniarm_vel/ServoControl.h"
#include <iostream>
// 0826

Manipulator* g_arm = nullptr;
// client for servo
ros::ServiceClient gripper_client;  

bool moveCallback(miniarm_vel::MoveArm::Request &req,
                  miniarm_vel::MoveArm::Response &res)
{
    ROS_INFO("Received target: x=%.2f, y=%.2f, z=%.2f", req.x, req.y, req.z); //收到座標訊息

    // 每次收到座標 → 自動打開夾爪
    miniarm_vel::ServoControl open_srv;
    open_srv.request.command = "open";
    if (gripper_client.call(open_srv)) {        // call server
        ROS_INFO("Gripper automatically opened: %s", open_srv.response.message.c_str());
    } else {
        ROS_WARN("Failed to call gripper open service");
    }
    //
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

        // 每次到達目標 → 關閉夾爪
        miniarm_vel::ServoControl close_srv;
        close_srv.request.command = "close"; 
        if (gripper_client.call(close_srv)) {       // call server
            ROS_INFO("Gripper closed: %s", close_srv.response.message.c_str());
        } else {
            ROS_ERROR("Failed to call gripper close service (maybe gripper node not running?)");
        }
        //

    } else {
        res.success = false;
        res.message = " Failed to reach target within step limit.";
        ROS_WARN("Failed to reach target.");
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_server");  //node name arm_server
    ros::NodeHandle nh;

    Manipulator arm;
    g_arm = &arm;
    // initialize to home position//////////////////////////////(08/29)
    Eigen::Matrix<float,3,1> home_position = g_arm->Home_Position();

    ROS_INFO_STREAM("Home position : x=" << home_position(0) 
                                << " y=" << home_position(1) 
                                << " z=" << home_position(2));

    bool reached = false;
    int max_steps = 5000;

    for (int i = 0; ros::ok() && i < max_steps; ++i) {
        reached = ag_arm->stepToward(0, 0, 0, home_position(0), home_position(1), home_position(2));
        ros::Duration(0.01).sleep();  // 100Hz
        if (reached) break;
    }
    if (reached) {
        ROS_INFO("Manipulator reached home position.");
    } else {
        ROS_WARN("Failed to reach home position within step limit.");
    }
    /////////////////////////////////////////////////////////////(08/29)

    // 建立 gripper_control client, 在 main 裡初始化 client
    gripper_client = nh.serviceClient<miniarm_vel::ServoControl>("gripper_control");

    // 建立 move_arm server
    ros::ServiceServer service = nh.advertiseService("move_arm", moveCallback);
    //
    ROS_INFO("Arm server ready. Waiting for client requests...");
    ros::spin();

    return 0;
}
