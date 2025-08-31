#include "ros/ros.h"
#include "miniarm_vel/MoveArm.h"
#include <iostream>
// 0826

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_client");
    ros::NodeHandle nh;

    ros::ServiceClient arm_client = nh.serviceClient<miniarm_vel::MoveArm>("move_arm");

    while (ros::ok()) {
        float x, y, z;
        std::cout << "Enter target position (x y z in mm): ";
        std::cin >> x >> y >> z;

        miniarm_vel::MoveArm srv;
        srv.request.x = x;
        srv.request.y = y;
        srv.request.z = z;

        if (client.call(srv)) {
            ROS_INFO("Response: success=%d, message=%s", srv.response.success, srv.response.message.c_str());
        } else {
            ROS_ERROR("Failed to call service move_arm");
        }
    }

    return 0;
}
