#include <ros/ros.h>
#include <fstream>
#include "miniarm_vel/Manipulator.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "test_trajectory");
  ros::NodeHandle nh;

  Manipulator arm;
  ros::Rate rate(100); // 100 Hz * 0.01s dt -> 同步

  // 你可以用 rosparam 改這些值
  float ox =  0.0f,  oy=0.0f,   oz=0.0f;             // 目標姿態 (deg)
  float px =  20.0f, py=20.0f, pz=5.0f;        // 目標位置 (mm)
  float rad2deg = 180.0f / M_PI;

  /*
  nh.getParam("target_ox_deg", ox);
  nh.getParam("target_oy_deg", oy);
  nh.getParam("target_oz_deg", oz);
  nh.getParam("target_px_mm", px);
  nh.getParam("target_py_mm", py);
  nh.getParam("target_pz_mm", pz);
  */

  // std::ofstream log("/tmp/trajectory_log1.csv");
  std::ofstream log("/root/catkin_ws/logs/trajectory_log.csv");
  log << "step,lin_err,ang_err,x,y,z\n";

  const int max_steps = 10000;  // maximum steps

  for (int i=0; ros::ok() && i<max_steps; ++i){
    bool done = arm.stepToward(ox, oy, oz, px, py, pz); // 是否閾值收斂?

    auto pos = arm.currentPosition();
    auto ori = arm.currentOrientation();
    Eigen::Vector3f tgt_o(ox*M_PI/180.0f, oy*M_PI/180.0f, oz*M_PI/180.0f);
    Eigen::Vector3f tgt_p(px, py, pz);
    Eigen::Matrix<float,6,1> q = arm.currentQ();

    // float lin_err = (tgt_p - pos).norm();
    float lin_err = arm.rmsErr(tgt_p, pos);
    float ang_err = (tgt_o - ori).norm()/std::sqrt(3.0f); 

    log << i << "," << lin_err << "," << ang_err << ","
        << pos(0) << "," << pos(1) << "," << pos(2) << "\n";
    if(i % 10 == 0){
        ROS_INFO("Joint angles: \n[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f](deg)",
         q(0)* rad2deg, q(1)* rad2deg, q(2)* rad2deg,
         q(3)* rad2deg, q(4)* rad2deg, q(5)* rad2deg);
     }
    if(i == 0){
        ROS_INFO("Initial Pose=(%.2f, %.2f, %.2f) mm"
      , pos(0), pos(1), pos(2));
    }
    if (done){
        ROS_INFO("Converged at step %d; Pos=(%.2f, %.2f, %.2f)",
               i, pos(0), pos(1), pos(2));
        ROS_INFO("Linear error: %.2f mm", lin_err);
        break;
}
    rate.sleep();
  }
  log.close();
  ROS_INFO("Log saved to /root/catkin_ws/logs/trajectory_log.csv");
  return 0;
}
