#include "ros/ros.h"
#include "miniarm_vel/ServoControl.h" 
#include <serial/serial.h>
// 0826

serial::Serial arduino_serial;

// Service Callback
bool handle_gripper_control(miniarm_vel::ServoControl::Request &req,
                            miniarm_vel::ServoControl::Response &res) {

    // 收到 command 就發給 Arduino
    arduino_serial.write(req.command + "\n");       // client 送來的指令（"open" 或 "close"）
    ROS_INFO("Gripper command sent: %s", req.command.c_str());

    res.success = true;
    res.message = "Command sent to gripper.";
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_control_server");  //node name gripper_control_server
    ros::NodeHandle nh("~");

    std::string port;
    int baud_rate;
    nh.param<std::string>("serial_port", port, "/dev/ttyACM0");
    nh.param<int>("baud_rate", baud_rate, 115200);

    // 初始化 serial
    arduino_serial.setPort(port);
    arduino_serial.setBaudrate(baud_rate);
    // arduino_serial.setTimeout(serial::Timeout::simpleTimeout(1000));
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    arduino_serial.setTimeout(to);
    try {
        arduino_serial.open();
    } catch (serial::IOException &e) {
        ROS_FATAL("Cannot open serial port %s: %s", port.c_str(), e.what());
        return -1;
    }

    // 確認連線是否成功
    if (!arduino_serial.isOpen()) {
        ROS_FATAL("Failed to open serial port %s", port.c_str());
        return -1;
    }
    ROS_INFO("Serial port %s opened at %d baud.", port.c_str(), baud_rate);

    // 建立 gripper_control server
    ros::ServiceServer service = nh.advertiseService("gripper_control", handle_gripper_control);
    ROS_INFO("Gripper Control Server ready.");

    ros::spin();
    return 0;
}
