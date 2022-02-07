/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Class to perform inverse geometry on a biped robot.
 */

#ifndef PREVIEW_IK_BIPED_IG
#define PREVIEW_IK_BIPED_IG

#include <Eigen/Dense>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "preview_ik/arm_ig.hpp"
#include "preview_ik/leg_ig.hpp"

namespace preview_ik {
/**
 * @brief @todo Describe BipedSettings
 */
struct BipedSettings {
 public:
  std::string left_hip_joint_name = "";
  std::string left_knee_joint_name = "";
  std::string left_ankle_joint_name = "";
  std::string left_foot_frame_name = "";
  std::string right_hip_joint_name = "";
  std::string right_knee_joint_name = "";
  std::string right_ankle_joint_name = "";
  std::string right_foot_frame_name = "";
  std::string urdf_path = "";
  std::string srdf_path = "";
};

/**
 * @brief @todo Describe BipedIG
 *
 */
class BipedIG {
  // Private attributes.
 private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  BipedSettings settings_;
  LegIG left_leg_, right_leg_;
  ArmIG left_arm_, right_arm_;
  Eigen::VectorXd q0_;
  Eigen::Vector3d com_from_waist_;

  // Private methods.
 private:
  void derivatives(const Eigen::VectorXd &q1, const Eigen::VectorXd &q3, Eigen::VectorXd &posture,
                   Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration, const double &dt);
  
  pinocchio::SE3 computeBase(const Eigen::Vector3d &com, const pinocchio::SE3 &leftFoot,
                             const pinocchio::SE3 &rightFoot);

  pinocchio::SE3 computeBase(const Eigen::Vector3d &com, const Eigen::Matrix3d &baseRotation);

  // Public methods.
 public:
  BipedIG();

  BipedIG(const BipedSettings &settings);

  void initialize(const BipedSettings &settings);

  const BipedSettings &get_settings();

  void checkCompatibility();  // TODO

  void configurateLegs();

  void solve(const Eigen::Vector3d &com, const pinocchio::SE3 &leftFoot, const pinocchio::SE3 &rightFoot,
             const Eigen::VectorXd &q0, Eigen::VectorXd &posture);

  void solve(const Eigen::Vector3d &com, const Eigen::Matrix3d &baseRotation, const pinocchio::SE3 &leftFoot,
             const pinocchio::SE3 &rightFoot, const Eigen::VectorXd &q0, Eigen::VectorXd &posture);

  void solve(const pinocchio::SE3 &base, const pinocchio::SE3 &leftFoot, const pinocchio::SE3 &rightFoot,
             const Eigen::VectorXd &q0, Eigen::VectorXd &posture);

  void solve(const std::array<Eigen::Vector3d, 3> &coms, const std::array<pinocchio::SE3, 3> &leftFeet,
             const std::array<pinocchio::SE3, 3> &rightFeet, const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
             Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration, const double &dt);

  void solve(const std::array<Eigen::Vector3d, 3> &coms, const std::array<Eigen::Matrix3d, 3> &baseRotations,
             const std::array<pinocchio::SE3, 3> &leftFeet, const std::array<pinocchio::SE3, 3> &rightFeet,
             const Eigen::VectorXd &q0, Eigen::VectorXd &posture, Eigen::VectorXd &velocity,
             Eigen::VectorXd &acceleration, const double &dt);

  void solve(const std::array<pinocchio::SE3, 3> &bases, const std::array<pinocchio::SE3, 3> &leftFeet,
             const std::array<pinocchio::SE3, 3> &rightFeet, const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
             Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration, const double &dt);

  Eigen::Vector2d computeCoP(const Eigen::VectorXd &posture, const Eigen::VectorXd &velocity,
                             const Eigen::VectorXd &acceleration, bool flatHorizontalGround = true);

  Eigen::Vector2d computeCoP(const Eigen::VectorXd &posture, const Eigen::VectorXd &velocity,
                             const Eigen::VectorXd &acceleration, const Eigen::Matrix<double, 6,1> &externalWrench,
                             bool flatHorizontalGround = true);
};
}  // namespace preview_ik
#endif  // PREVIEW_IK_BIPED_IG