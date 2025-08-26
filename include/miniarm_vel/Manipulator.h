#pragma once
#include <Eigen/Dense>
#include "../MotorUnion/MotorUnion.h"

// Manipulator 繼承 MotorUnion
class Manipulator : public MotorUnion {
public:
  Manipulator();
  ~Manipulator();

  // 一步一步朝目標前進；回傳
  bool stepToward(float ox_deg, float oy_deg, float oz_deg,
                  float px, float py, float pz);

  // 停止所有馬達
  void Stop();

  // 取得目前末端姿態與位置（姿態為 radians, ZYX）
  Eigen::Matrix<float,3,1> currentOrientation() const { return current_orientation_; }
  Eigen::Matrix<float,3,1> currentPosition()    const { return current_position_;   }
  Eigen::Matrix<float,6,1> currentQ()           const { return q_; }                  // 六關節角度 (rad)

  // 工具函式
  float rmsErr(const Eigen::Matrix<float,3,1>& t,
               const Eigen::Matrix<float,3,1>& c);

private:
  // 幾何/運動學
  Eigen::Matrix<float,4,4> T(float th, float alpha, float a, float d);
  void computeKinematicsAndJacobian();

  // Damped Least Squares： (J^T (J J^T + λ^2 I)^-1) v
  Eigen::Matrix<float,6,1> solveDLS(const Eigen::Matrix<float,6,6>& J,
                                    const Eigen::Matrix<float,6,1>& v,
                                    float lambda);

  // 內部狀態
  Eigen::Matrix<float,6,1> q_;             // 六關節角度 (rad)
  Eigen::Matrix<float,6,6> J_;             // Jacobian（上=angular, 下=linear）
  Eigen::Matrix<float,3,1> current_position_;
  Eigen::Matrix<float,3,1> current_orientation_;

  // 參數（可調）
  float dt_               = 0.01f;   // 內部離散步長（秒）
  float vel_gain_         = 5.0f;    // 比例增益（對 end-effector 速度）
  float lambda_dls_       = 0.05f;   // DLS 阻尼，避免奇異點暴衝
  float angular_thresh_   = 5 * M_PI/ 180.0f; // 5°
  float linear_thresh_mm_ = 0.15f;    // 0.15 mm
  float joint_speed_max_  = M_PI/3;  // 每關節安全上限(rad/s) ~60°/s

  // DH 參數（依你提供的「小手臂」）
  float a_[6]     = {0, 0, 24.5f, 0, 0, 0};         // (mm)
  float d_[6]     = {0, 0, 0, 30.4f, 0, 0};         // (mm)
  float alpha_[6] = {0, -M_PI/2, 0, -M_PI/2, M_PI/2, -M_PI/2};
};
