/**
 * @file
 * @copyright Copyright (c) 2022, LAAS-CNRS, Toward, PalRobotics
 * @brief
 */

#include "aig/leg_ig.hpp"

namespace aig {

LegIG::LegIG() { reset_internals(); }

LegIG::LegIG(const LegIGSettings &settings) { initialize(settings); }

void LegIG::initialize(const LegIGSettings &settings) {
  settings_ = settings;
  reset_internals();
}

void LegIG::reset_internals() {
  // Small quantity to prevent numerical issues.
  epsilon_ = 1.0e-6;
  hip_ = ankle_ = hip_from_ankle_ = Eigen::Vector3d::Zero();
  c5_ = 0.0;
  q2_ = q3_ = q4_ = q5_ = q6_ = q7_ = 0.0;
  opp_sign_hip_from_waist_y_ = sign_hip_from_ankle_z = 0.0;
  a_ = b_ = c_ = 0.0;
  Rint_ = Rext_ = R_ = Eigen::Matrix3d::Zero();
  output_ = LegJoints::Zero();
}

LegJoints LegIG::solve(const pinocchio::SE3 &base,
                       const pinocchio::SE3 &endEffector) {
  reset_internals();

  // First we compute the position of the hip with respect to the ankle.
  hip_ = base.translation() + base.rotation() * settings_.hip_from_waist;
  ankle_ = endEffector.translation() +
           endEffector.rotation() * settings_.ankle_from_foot;
  hip_from_ankle_ = endEffector.rotation().transpose() * (hip_ - ankle_);

  // if hip_from_waist(1)<0.0 then out=1.0 else out=-1.0
  opp_sign_hip_from_waist_y_ = settings_.hip_from_waist(1) < 0.0 ? 1.0 : -1.0;

  // Compute the cos(q5)
  const Eigen::Vector3d &knee_from_hip = settings_.knee_from_hip;
  const Eigen::Vector3d &ankle_from_knee = settings_.ankle_from_knee;
  a_ = abs(knee_from_hip(2));    // Femur Height.
  b_ = abs(ankle_from_knee(2));  // Tibia Height.
  c_ = sqrt(hip_from_ankle_(0) * hip_from_ankle_(0) +
            hip_from_ankle_(2) * hip_from_ankle_(2));
  c5_ = 0.5 * (c_ * c_ - a_ * a_ - b_ * b_) / (a_ * b_);
  // Compute q5 (the knee).
  if (c5_ > 1.0 - epsilon_) {
    q5_ = 0.0;
  }
  if (c5_ < -1.0 + epsilon_) {
    q5_ = M_PI;
  }
  if (c5_ >= -1.0 + epsilon_ && c5_ <= 1.0 - epsilon_) {
    q5_ = acos(c5_);
  }

  // Compute the orientation of the ankle.
  sign_hip_from_ankle_z = hip_from_ankle_(2) > 0 ? 1.0 : -1.0;
  q6_ = -atan2(hip_from_ankle_(0),
               sign_hip_from_ankle_z * hip_from_ankle_.tail<2>().norm()) -
        asin(a_ * sin(M_PI - q5_) / c_);

  q7_ = atan2(hip_from_ankle_(1), hip_from_ankle_(2));
  if (q7_ > M_PI_2) {
    q7_ -= M_PI;
  } else if (q7_ < -M_PI_2) {
    q7_ += M_PI;
  }

  Rext_ = base.rotation().transpose() * endEffector.rotation();
  Rint_ = Eigen::AngleAxisd(-q7_, Eigen::Vector3d(1, 0, 0)) *
          Eigen::AngleAxisd(-q5_ - q6_, Eigen::Vector3d(0, 1, 0));
  R_ = Rext_ * Rint_;
  q2_ = atan2(-R_(0, 1), R_(1, 1));
  q3_ = atan2(R_(2, 1), -R_(0, 1) * sin(q2_) + R_(1, 1) * cos(q2_));
  q4_ = atan2(-R_(2, 0), R_(2, 2));

  output_ << q2_, q3_, q4_, q5_, q6_, q7_;
  return output_;
}

}  // namespace aig
