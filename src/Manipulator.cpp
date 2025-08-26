#include "miniarm_vel/Manipulator.h"
#include "../MotorUnion/MotorUnion.h"
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>
#include <iostream>
// 0826

static inline float deg2rad(float d){ return d * M_PI / 180.0f; }


Manipulator::Manipulator()
  : MotorUnion({1, 2, 3, 4, 5, 6}, {"Mx106","Mx106","Mx106","Mx106","Mx106","Mx106"})
{
  q_.setZero();
  current_position_.setZero();
  current_orientation_.setZero();

  SetAllMotorsOperatingMode(1); // 1 = velocity mode
  SetAllMotorsTorqueEnable(true);

  std::cout << "Manipulator constructed & communication ready" << std::endl;

  computeKinematicsAndJacobian();
}
/*
Manipulator::Manipulator() {
  q_ << deg2rad(0.0f), deg2rad(20.0f), deg2rad(20.0f),
        deg2rad(0.0f), deg2rad(20.0f), deg2rad(0.0f);  

  computeKinematicsAndJacobian();  
}
*/

Manipulator::~Manipulator() {
  Stop();
}
void Manipulator::Stop() {
  SetAllMotorsTorqueEnable(false);
  std::cout << "Manipulator stopped (torque disabled)" << std::endl;
}

Eigen::Matrix<float,4,4> Manipulator::T(float th, float al, float a, float d) {
  Eigen::Matrix<float,4,4> m;
  m << cosf(th), -cosf(al)*sinf(th),  sinf(al)*sinf(th), a*cosf(th),
       sinf(th),  cosf(al)*cosf(th), -sinf(al)*cosf(th), a*sinf(th),
       0,         sinf(al),           cosf(al),          d,
       0,         0,                  0,                 1;
  return m;
}

float Manipulator::rmsErr(const Eigen::Vector3f& t,
                          const Eigen::Vector3f& c) {
    Eigen::Vector3f diff = t - c;
    return std::sqrt(diff.squaredNorm() / diff.size());
}


Eigen::Matrix<float,6,1> Manipulator::solveDLS(const Eigen::Matrix<float,6,6>& J,
                                               const Eigen::Matrix<float,6,1>& v,
                                               float lambda){
  Eigen::Matrix<float,6,6> A = J * J.transpose() + lambda*lambda*Eigen::Matrix<float,6,6>::Identity();
  Eigen::Matrix<float,6,6> Ainv = A.inverse();
  return J.transpose() * (Ainv * v);
}

void Manipulator::computeKinematicsAndJacobian() {
  float J0=q_(0), J1=q_(1), J2=q_(2), J3=q_(3), J4=q_(4), J5=q_(5);

  Eigen::Matrix<float,4,4> T01 = T(J0, alpha_[0], a_[0], d_[0]);
  Eigen::Matrix<float,4,4> T12 = T(J1, alpha_[1], a_[1], d_[1]);
  Eigen::Matrix<float,4,4> T23 = T(J2, alpha_[2], a_[2], d_[2]);
  Eigen::Matrix<float,4,4> T34 = T(J3, alpha_[3], a_[3], d_[3]);
  Eigen::Matrix<float,4,4> T45 = T(J4, alpha_[4], a_[4], d_[4]);
  Eigen::Matrix<float,4,4> T56 = T(J5, alpha_[5], a_[5], d_[5]);

  /*
  Eigen::Matrix4f T_base = Eigen::Matrix4f::Identity();
  T_base(0,0)=-1; T_base(1,1)=1; T_base(2,2)=1;
  T01 = T_base * T01;
  */
  

  // 先轉成 Eigen 矩陣
  Eigen::Matrix4f B0;
  B0 << 1,0,0,0,
        0,1,0,0,
        0,0,1,15.4,  // 基座高度
        0,0,0,1;

  Eigen::Matrix4f E6;
  E6 << 1,0,0,0,
        0,1,0,0,
        0,0,1,20,    // 末端執行器長度
        0,0,0,1;

  Eigen::Matrix<float,4,4> T02=T01*T12, T03=T02*T23, T04=T03*T34,
                           T05=T04*T45, T06=T05*T56;
                           // T06 = B0 * T06 ;
                           T06 = B0 * T06 * E6 ;
                        


  // 末端姿態（ZYX Euler，與你一致）
  float oz = atan2(T06(1,0), T06(0,0));
  float oy = atan2(-T06(2,0), (T06(0,0)*cosf(oz) + T06(1,0)*sinf(oz)));
  float ox = atan2((T06(0,2)*sinf(oz) - T06(1,2)*cosf(oz)), (T06(1,1)*cosf(oz) - T06(0,1)*sinf(oz)));
  current_orientation_ << ox, oy, oz;
  current_position_ << T06(0,3), T06(1,3), T06(2,3); // (mm)

  // 取 Z 軸與各原點位置
  Eigen::Vector3f P00(0,0,0),
                  P01(T01(0,3),T01(1,3),T01(2,3)),
                  P02(T02(0,3),T02(1,3),T02(2,3)),
                  P03(T03(0,3),T03(1,3),T03(2,3)),
                  P04(T04(0,3),T04(1,3),T04(2,3)),
                  P05(T05(0,3),T05(1,3),T05(2,3)),
                  P06(T06(0,3),T06(1,3),T06(2,3));

  Eigen::Vector3f Z00(0,0,1),
                  Z01(T01(0,2),T01(1,2),T01(2,2)),
                  Z02(T02(0,2),T02(1,2),T02(2,2)),
                  Z03(T03(0,2),T03(1,2),T03(2,2)),
                  Z04(T04(0,2),T04(1,2),T04(2,2)),
                  Z05(T05(0,2),T05(1,2),T05(2,2));

  Eigen::Vector3f Jv00 = Z00.cross(P06-P00);
  Eigen::Vector3f Jv01 = Z01.cross(P06-P01);
  Eigen::Vector3f Jv02 = Z02.cross(P06-P02);
  Eigen::Vector3f Jv03 = Z03.cross(P06-P03);
  Eigen::Vector3f Jv04 = Z04.cross(P06-P04);
  Eigen::Vector3f Jv05 = Z05.cross(P06-P05);

  // 上=角速度， 下=線速度（配合你原本 end-effector 速度排列）
  J_ <<
    Z00(0), Z01(0), Z02(0), Z03(0), Z04(0), Z05(0),
    Z00(1), Z01(1), Z02(1), Z03(1), Z04(1), Z05(1),
    Z00(2), Z01(2), Z02(2), Z03(2), Z04(2), Z05(2),
    Jv00(0), Jv01(0), Jv02(0), Jv03(0), Jv04(0), Jv05(0),
    Jv00(1), Jv01(1), Jv02(1), Jv03(1), Jv04(1), Jv05(1),
    Jv00(2), Jv01(2), Jv02(2), Jv03(2), Jv04(2), Jv05(2);
}

bool Manipulator::stepToward(float ox_deg, float oy_deg, float oz_deg,
                             float px, float py, float pz) {
  // 目標：姿態（deg → rad），位置（mm）
  Eigen::Vector3f target_o(deg2rad(ox_deg), deg2rad(oy_deg), deg2rad(oz_deg));
  Eigen::Vector3f target_p(px, py, pz);

  // 更新目前 kinematics/Jacobian
  computeKinematicsAndJacobian();

  float ang_err = rmsErr(target_o, current_orientation_);
  float lin_err = rmsErr(target_p, current_position_);

  //if (ang_err < angular_thresh_ && lin_err < linear_thresh_mm_) return true;
  if (lin_err < linear_thresh_mm_) return true;

  static float accel_factor = 0.8f; 
  static int accel_counter = 0;

  // P 控制：end-effector 速度（角、線）
  Eigen::Vector3f w = (target_o - current_orientation_) * vel_gain_ * accel_factor;
  Eigen::Vector3f v = (target_p - current_position_)   * vel_gain_ * accel_factor;

  if (accel_factor < 1.0f && accel_counter % 20 == 0) accel_factor += 0.1f;
  accel_counter++;

  Eigen::Matrix<float,6,1> xdot;
  xdot << w, v; // 配合 J 的列順序

  // Joint 速度：DLS 解，避免奇異點
  // Rotation speed
  Eigen::Matrix<float,6,1> qdot = solveDLS(J_, xdot, lambda_dls_);

  // upper bond
  float max_abs = qdot.cwiseAbs().maxCoeff();
  if (max_abs > joint_speed_max_) qdot *= (joint_speed_max_/max_abs);

  // lower bound
  float lower_limit = 1e-3f; // "可根據馬達精度調整"
  for (int i = 0; i < 6; i++) {
      if (std::abs(qdot(i)) < lower_limit) qdot(i) = 0.0f;
  }

  // 積分更新
  q_ +=  qdot * dt_;

  // Communication set_motor_velocity////////////////////////////////////
  for (int i = 0; i < 6; i++) {
      double motor_speed = qdot(i) * 60.0 / (2*M_PI); // rad/s → RPM
      SetMotor_Velocity(1 + i, motor_speed);
  }
  ///////////////////////////////////////////////////////////////////////

  static int stop_counter = 0;
  if (qdot.cwiseAbs().maxCoeff() < 0.001f) { 
      stop_counter++;
      if (stop_counter >= 20) { 
          stop_counter = 0; accel_factor = 0.5f; // reset
          return true; 
      }
  } else {
      stop_counter = 0; 
  }
  return false;
}
