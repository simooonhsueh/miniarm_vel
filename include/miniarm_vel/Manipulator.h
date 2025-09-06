#pragma once
#include <Eigen/Dense>
#include "../MotorUnion/MotorUnion.h"
// 0826

inline float deg2rad(float deg) { return deg * M_PI / 180.0f; }
// inline float rad2deg(float rad) { return rad * 180.0f/ M_PI;  }

// Manipulator 繼承 MotorUnion
class Manipulator : public MotorUnion {
public:
  Manipulator();
  ~Manipulator();

  const float pro_radpersec2scale_;
  
  // 一步一步朝目標前進；回傳
  bool stepToward(float ox_deg, float oy_deg, float oz_deg,
                  float px, float py, float pz);
  void debugKinematics();
  
  void UpdateJointAnglesFromMotors(); // "08/28"
  void GetHome();                     // "08/29"


  // 停止所有馬達
  void Stop();

  // 取得目前末端姿態與位置（姿態為 radians, ZYX）
  Eigen::Quaternionf current_orientation_q_; //0906
  Eigen::Matrix<float,3,1> currentOrientation() const { return current_orientation_; }
  Eigen::Matrix<float,3,1> currentPosition()    const { return current_position_;   }
  Eigen::Matrix<float,6,1> currentQ()           const { return q_; }                   // 六關節角度 (rad)
  Eigen::Matrix<float,6,1> Endeffector_Position(float q0, float q1, float q2, float q3, float q4, float q5); // "08/29"
  Eigen::Matrix<float,6,1> Home_Position();      // "08/29"
  Eigen::Matrix<float,6,1> CheckMotorVelocity(Eigen::Matrix<float, 6, 1> motor_velocity);
  static Eigen::Matrix<float,6,1> prev_qdot;
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
  bool is_out_of_limit_;

  // 內部狀態
  Eigen::Matrix<float,6,1> q_;                           // 六關節角度 (rad)
  Eigen::Matrix<float,6,6> Jacobian_matrix_;             // Jacobian（上=angular, 下=linear）
  Eigen::Matrix<float,3,1> current_position_;
  Eigen::Matrix<float,3,1> current_orientation_;
  Eigen::Matrix<float,6,1> motor_velocity;

  // 參數（可調）
  float dt_               = 0.01f;   // 內部離散步長（秒）
  float vel_gain_         = 5.0f;    // 比例增益（對 end-effector 速度）
  float lambda_dls_       = 0.1f;   // DLS 阻尼，避免奇異點暴衝
  float angular_thresh_   = 20 * M_PI/ 180.0f; // 10°
  float linear_thresh_mm_ = 50.0f;   // 0.15 mm
  float joint_speed_max_  = M_PI/3;  // 每關節安全上限(rad/s) ~60°/s

  Eigen::Matrix<float,6,1> q_home_ = (Eigen::Matrix<float,6,1>() <<
                                      deg2rad(180.26f), deg2rad(408.92f), deg2rad(199.16f),
                                      deg2rad(3.38f),   deg2rad(108.35f), deg2rad(208.31f)).finished(); // rad

  // DH Table（「小手臂」）
  float a_[6]     = {0, 0, 245.0f, 0, 0, 0};                   // (mm)
  float d_[6]     = {0, 0, 0, 302.0f, 0, 0};                   // (mm)
  float alpha_[6] = {0, -M_PI/2, 0, -M_PI/2, M_PI/2, -M_PI/2}; // (rad)
  float theta_offset_[6] = {deg2rad(-180.0f), deg2rad(-186.33f), deg2rad(-178.77f), deg2rad(0.0f), deg2rad(0.0f), deg2rad(-38.94f)}; // (rad)
  
  float base_height_ = 100.0f; // (mm)
  float end_length_  = 110.0f; // (mm)
};
