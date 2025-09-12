#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "miniarm_vel/SendPosition.h"   // 新的 srv
#include <iostream>

geometry_msgs::Point latest_point;
bool has_point = false;

// callback: 訂閱座標
void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    latest_point = msg->point;
    has_point = true;
    ROS_INFO("Received point: x=%.2f, y=%.2f, z=%.2f",
             msg->point.x, msg->point.y, msg->point.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_position_client");
    ros::NodeHandle nh;

    // 訂閱座標
    ros::Subscriber sub = nh.subscribe("/selected_object/position", 10, positionCallback); //訂閱來自鏡頭辨識到的三個數字float

    // 建立 service client，對應到新的 server
    ros::ServiceClient client = nh.serviceClient<miniarm_vel::SendPosition>("send_position");

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        if (has_point) {
            std::string input;
            std::cout << "Type 'yes' to send position to server: ";
            std::cin >> input;

            if (input == "yes") {
                miniarm_vel::SendPosition srv;
                srv.request.x = latest_point.x;
                srv.request.y = latest_point.y;
                srv.request.z = latest_point.z;

                if (client.call(srv)) {
                    ROS_INFO("Server response: success=%d, message=%s",
                             srv.response.success, srv.response.message.c_str());
                } else {
                    ROS_ERROR("Failed to call send_position service");
                }
            }
        }

        loop_rate.sleep();
    }

    return 0;
}
