#include "ros/ros.h"
#include "miniarm_vel/MoveArm.h"
#include "miniarm_vel/Manipulator.h"
#include "miniarm_vel/ServoControl.h"
#include "miniarm_vel/SendPosition.h"
#include <iostream>
// 0826
float rad2deg = 180.0f / M_PI;
Manipulator* g_arm = nullptr;
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
    bool reached1 = false;
    int max_steps = 5000;

    for (int i = 0; ros::ok() && i < max_steps; ++i) {

        Eigen::Matrix<float,6,1> q = g_arm->currentQ(); // get current joint angles //(rad)
        Eigen::Vector2f lin_ang_error = g_arm->currentError();
        reached1 = g_arm->stepToward(0, 200, 0, req.x, req.y, req.z);
        
        if(i % 50 == 0){
        ROS_INFO("Joint angles: \n[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f](deg)",
         q(0)* rad2deg, q(1)* rad2deg, q(2)* rad2deg,
         q(3)* rad2deg, q(4)* rad2deg, q(5)* rad2deg);
         ROS_INFO("Linear error: [%.2f]mm, Angular error: [%.2f]deg", lin_ang_error[0] , lin_ang_error[1] * rad2deg);
        }
        
        ros::Duration(0.01).sleep(); // 100 Hz
        if (reached1) break;
    }

    if (reached1) {
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

// 09/12
// 新的server函式，可以用於接收相機辨識的座標，完成（EE—>物件的相對位置差）
// callback: 接收 client 送來的座標
bool handleSendPosition(miniarm_vel::SendPosition::Request &req,
                        miniarm_vel::SendPosition::Response &res)
{
    Eigen::Matrix<float,6,1> home_position = g_arm->Home_Position();
    ROS_INFO("Received position from client: x=%.2f, y=%.2f, z=%.2f",
             req.x, req.y, req.z);
    Eigen::Matrix<float,3,1> relative_position;
    relative_position[0] = home_position[0] - req.x;
    relative_position[1] = home_position[1] - req.y;
    relative_position[2] = home_position[2] - req.z;
    ROS_INFO("Object position: [%.2f, %.2f, %.2f](mm)",
            relative_position[0], relative_position[1], relative_position[2]);

    bool reached2 = false;
    int max_steps = 5000;
    for (int i = 0; ros::ok() && i < max_steps; ++i) {

        Eigen::Matrix<float,6,1> q = g_arm->currentQ(); 
        Eigen::Vector2f lin_ang_error = g_arm->currentError();

        reached2 = g_arm->stepToward(0, 200, 0, relative_position[0], relative_position[1], relative_position[2]);
        if(i% 50 == 0){
            ROS_INFO("Joint angles: \n[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f](deg)",
            q(0)* rad2deg, q(1)* rad2deg, q(2)* rad2deg,
            q(3)* rad2deg, q(4)* rad2deg, q(5)* rad2deg);
            ROS_INFO("Linear error: [%.2f]mm, Angular error: [%.2f]deg", lin_ang_error[0] , lin_ang_error[1] * rad2deg);
        }
        ROS_INFO("Linear error: [%.2f]mm, Angular error: [%.2f]deg", lin_ang_error[0] , lin_ang_error[1] * rad2deg);
        
        ros::Duration(0.01).sleep();  // 100Hz
        if (reached2) break;
        
    }
    if (reached2) {
        ROS_INFO("Manipulator reached home position.");
    } else {
        ROS_WARN("Failed to reach home position within step limit.");
    }
    res.success = true;
    res.message = "Position received and processed.";
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_server");  //node name arm_server
    ros::NodeHandle nh;

    Manipulator arm;
    g_arm = &arm;
    // initialize to home position//////////////////////////////(08/29)
    Eigen::Matrix<float,6,1> home_position = g_arm->Home_Position();

    bool reached = false;
    int max_steps = 5000;

    for (int i = 0; ros::ok() && i < max_steps; ++i) {

        // g_arm->debugKinematics();
        reached = g_arm->stepToward(home_position(3)* rad2deg, home_position(4)* rad2deg, home_position(5)* rad2deg, home_position(0), home_position(1), home_position(2));
        Eigen::Vector2f lin_ang_error = g_arm->currentError();
        if(i % 50 == 0){
        Eigen::Matrix<float,6,1> q = g_arm->currentQ(); // get current joint angles //rad
        ROS_INFO("Joint angles: \n[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f](deg)",
            q(0)* rad2deg, q(1)* rad2deg, q(2)* rad2deg,
            q(3)* rad2deg, q(4)* rad2deg, q(5)* rad2deg);
        Eigen::Matrix<float,6,1> currentt_position = g_arm->Endeffector_Position(q(0), q(1), q(2), q(3), q(4), q(5));
        // ROS_INFO("Current position:    [%.2f, %.2f, %.2f]mm", currentt_position(0), currentt_position(1), currentt_position(2));  //0904_testing
        // ROS_INFO("Current orientation: [%.2f, %.2f, %.2f]deg", currentt_position(3)*rad2deg, currentt_position(4)*rad2deg, currentt_position(5)*rad2deg); //0904_testing
        // ROS_INFO("Initial position:    [%.2f, %.2f, %.2f]mm", home_position(0), home_position(1), home_position(2));
        // ROS_INFO("Initial orientation: [%.2f, %.2f, %.2f]deg", home_position(3)*rad2deg, home_position(4)*rad2deg, home_position(5)*rad2deg);
        ROS_INFO("Linear error: [%.2f]mm, Angular error: [%.2f]deg", lin_ang_error[0] , lin_ang_error[1] * rad2deg);
        }
        ros::Duration(0.01).sleep();  // 100Hz
        if (reached) break;
    }
    if (reached) {
        ROS_INFO("Manipulator reached home position.");
    } else {
        ROS_WARN("Failed to reach home position within step limit.");
    }
    /////////////////////////////////////////////////////////////

    // 建立 gripper_control client, 在 main 裡初始化 client
    gripper_client = nh.serviceClient<miniarm_vel::ServoControl>("/gripper_control"); //08/31

    // 建立 move_arm server, 在 main 裡初始化 server
    ros::ServiceServer arm_service = nh.advertiseService("move_arm", moveCallback);

    // 建立 position_server
    ros::ServiceServer position_service = nh.advertiseService("send_position", handleSendPosition);

    ROS_INFO("Arm server ready. Waiting for client requests...");
    ros::spin();

    return 0;
}

