/**
 * @file
 * @copyright Copyright (c) 2022, LAAS-CNRS, Toward, PalRobotics
 * @brief
 */

#include "aig/contact6d.hpp"
#include "pinocchio/algorithm/geometry.hpp"

namespace aig {

Contact6D::Contact6D() {}
Contact6D::Contact6D(const Contact6DSettings &settings) {
  initialize(settings);
}
void Contact6D::initialize(const Contact6DSettings &settings) {
  settings_ = settings;
  double hl = settings_.half_length;
  double hw = settings_.half_width;
  double mu = settings_.mu;
  double gu = settings_.gu;

  // Make matrices
  regularization_A_ << settings_.weights;
  regularization_b_ << Eigen::Matrix<double, 6, 1>::Zero();

  unilaterality_A_ << Eigen::Matrix<double, 5, 6>::Zero();
  unilaterality_A_.block<5, 1>(0, 2) << -1, -hl, -hw, -hl, -hw;
  unilaterality_A_.block<2, 2>(1, 3) << 0, -1, 1, 0;
  unilaterality_A_.block<2, 2>(3, 3) << 0, 1, -1, 0;
  unilaterality_b_ << Eigen::Matrix<double, 5, 1>::Zero();

  friction_A_ << Eigen::Matrix<double, 6, 6>::Zero();
  friction_A_.block<6, 1>(0, 2) << -mu, -mu, -mu, -mu, -gu, -gu;
  friction_A_.block<2, 2>(0, 0) << Eigen::Matrix2d::Identity();
  friction_A_.block<2, 2>(2, 0) << -Eigen::Matrix2d::Identity();
  friction_A_.block<2, 1>(4, 5) << 1, -1;
  friction_b_ << Eigen::Matrix<double, 6, 1>::Zero();

  newton_euler_A_ << Eigen::Matrix<double, 6, 6>::Zero();
  newton_euler_A_.block<3, 3>(0, 0) << Eigen::Matrix3d::Identity();
  newton_euler_A_.block<3, 3>(3, 3) << Eigen::Matrix3d::Identity();

  // Note: newton_euler must be updated before using it
  contactForce_ = Eigen::Matrix<double, 6, 1>::Zero();
}

void Contact6D::setForceWeights(const Eigen::Vector3d &force_weights) {
  settings_.weights.head<3>() = force_weights;
  regularization_A_.head<3>() = force_weights;
}

void Contact6D::setTorqueWeights(const Eigen::Vector3d &torque_weights) {
  settings_.weights.tail<3>() = torque_weights;
  regularization_A_.tail<3>() = torque_weights;
}

void Contact6D::setSurfaceHalfWidth(const double &half_width) {
  settings_.half_width = half_width;
  unilaterality_A_(2, 2) = -half_width;
  unilaterality_A_(4, 2) = -half_width;
}

void Contact6D::setSurfaceHalfLength(const double &half_length) {
  settings_.half_length = half_length;
  unilaterality_A_(1, 2) = -half_length;
  unilaterality_A_(3, 2) = -half_length;
}

void Contact6D::setMu(const double &mu) {
  settings_.mu = mu;
  friction_A_.block<4, 1>(0, 2) << -mu, -mu, -mu, -mu;
}

void Contact6D::setGu(const double &gu) {
  settings_.gu = gu;
  friction_A_.block<2, 1>(4, 2) << -gu, -gu;
}

void Contact6D::updateNewtonEuler(const Eigen::Vector3d &CoM,
                                  const pinocchio::SE3 &oMs) {
  /**
   * @brief Assuming that the orientation of the world frame is the identity.
   *
   */

  oMs_ = oMs;
  cMo_ = pinocchio::SE3(Eigen::Matrix3d::Identity(), -CoM);

  newton_euler_A_ << (cMo_.act(oMs_)).toActionMatrixInverse().transpose();
}
}  // namespace aig
