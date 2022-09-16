/**
 * @file
 * @copyright Copyright (c) 2022, LAAS-CNRS, Toward, PalRobotics
 * @brief
 */

#include "aig/biped_ig.hpp"

#include <algorithm>
#include <cctype>

#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "aig/c_dynamics.hpp"

namespace aig {

BipedIGSettings makeSettingsFor(const std::string &path_to_robots,
                                const std::string &robot_name) {
  BipedIGSettings robot_settings;
  std::string robot_name_lower = robot_name;
  std::transform(robot_name_lower.begin(), robot_name_lower.end(),
                 robot_name_lower.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (robot_name_lower == "talos") {
    robot_settings.urdf = path_to_robots + "/robots/talos_reduced.urdf";
    robot_settings.srdf = path_to_robots + "/srdf/talos.srdf";

    robot_settings.left_hip_joint_name = "leg_left_1_joint";
    robot_settings.right_hip_joint_name = "leg_right_1_joint";
    robot_settings.left_knee_joint_name = "leg_left_4_joint";
    robot_settings.right_knee_joint_name = "leg_right_4_joint";
    robot_settings.left_ankle_joint_name = "leg_left_5_joint";
    robot_settings.right_ankle_joint_name = "leg_right_5_joint";
    robot_settings.left_foot_frame_name = "leg_left_sole_fix_joint";
    robot_settings.right_foot_frame_name = "leg_right_sole_fix_joint";
  } else {
    throw std::runtime_error(
        "biped_ig::aig::make_settings_for: No default settings for "
        "a robot with this name were specified yet.");
  }
  return robot_settings;
}

BipedIG::BipedIG() {
  // initialized by default:
  // model_
  // data_
  // settings_
  // left_leg_
  // right_leg_
  // left_arm_
  // right_arm_
}

BipedIG::BipedIG(const BipedIGSettings &settings) { initialize(settings); }

void BipedIG::initialize(const BipedIGSettings &settings) {
  // Copy the settings internally.
  settings_ = settings;

  // Check if the urdf and the srdf file exists or not.
  bool urdf_file_exists = false;
  bool srdf_file_exists = false;
  {
    std::ifstream f(settings_.urdf.c_str());
    urdf_file_exists = f.good();
  }
  {
    std::ifstream f(settings_.srdf.c_str());
    srdf_file_exists = f.good();
  }

  // Build the robot model.
  if (urdf_file_exists) {
    pinocchio::urdf::buildModel(settings_.urdf,
                                pinocchio::JointModelFreeFlyer(), model_);
  } else if (settings_.urdf != "") {
    pinocchio::urdf::buildModelFromXML(
        settings_.urdf, pinocchio::JointModelFreeFlyer(), model_);
  } else {
    throw std::runtime_error("BipedIG::BipedIG(): settings_.urdf is empty");
  }
  // Build pinocchio cache.
  data_ = pinocchio::Data(model_);

  gravity_ = model_.gravity981;
  mass_ = 0.0;
  for (size_t k = 0; k < model_.inertias.size(); ++k) {
    mass_ += model_.inertias[k].mass();
  }
  weight_ = mass_ * gravity_;
  S_ << 0, -1, 1, 0;

  // Extract the CoM to Waist level arm.
  if (srdf_file_exists) {
    pinocchio::srdf::loadReferenceConfigurations(model_, settings_.srdf, false);
  } else if (settings_.srdf != "") {
    std::stringstream buffer;
    buffer << settings_.srdf;
    pinocchio::srdf::loadReferenceConfigurationsFromXML(model_, buffer, false);
  } else {
    throw std::runtime_error("BipedIG::BipedIG(): settings_.srdf is empty");
  }
  q0_ = model_.referenceConfigurations["half_sitting"];
  set_com_from_waist(q0_);

  configureLegs();

  dyno::DynoSettings dyn_settings;
  dyn_settings.urdf = settings_.urdf;
  dynamics_ = dyno::Dyno(dyn_settings);

}

void BipedIG::configureLegs() {
  LegIGSettings left_leg_settings, right_leg_settings;
  pinocchio::JointIndex left_hip_id =
      model_.getJointId(settings_.left_hip_joint_name);
  pinocchio::JointIndex left_knee_id =
      model_.getJointId(settings_.left_knee_joint_name);
  pinocchio::JointIndex left_ankle_id =
      model_.getJointId(settings_.left_ankle_joint_name);
  pinocchio::JointIndex right_hip_id =
      model_.getJointId(settings_.right_hip_joint_name);
  pinocchio::JointIndex right_knee_id =
      model_.getJointId(settings_.right_knee_joint_name);
  pinocchio::JointIndex right_ankle_id =
      model_.getJointId(settings_.right_ankle_joint_name);

  pinocchio::FrameIndex leftSoleID =
      model_.getFrameId(settings_.left_foot_frame_name);
  pinocchio::FrameIndex rightSoleID =
      model_.getFrameId(settings_.right_foot_frame_name);

  left_leg_settings.hip_from_waist =
      model_.jointPlacements[left_hip_id].translation();
  left_leg_settings.knee_from_hip =
      model_.jointPlacements[left_knee_id].translation();
  left_leg_settings.ankle_from_knee =
      model_.jointPlacements[left_ankle_id].translation();
  left_leg_settings.ankle_from_foot =
      -model_.frames[leftSoleID].placement.translation();
  left_leg_.initialize(left_leg_settings);

  right_leg_settings.hip_from_waist =
      model_.jointPlacements[right_hip_id].translation();
  right_leg_settings.knee_from_hip =
      model_.jointPlacements[right_knee_id].translation();
  right_leg_settings.ankle_from_knee =
      model_.jointPlacements[right_ankle_id].translation();
  right_leg_settings.ankle_from_foot =
      -model_.frames[rightSoleID].placement.translation();
  right_leg_.initialize(right_leg_settings);

  // Get the legs joints configuration for the test
  lleg_idx_qs_ = model_.idx_qs[left_hip_id];
  rleg_idx_qs_ = model_.idx_qs[right_hip_id];
}

pinocchio::SE3 BipedIG::computeBase(const Eigen::Vector3d &com,
                                    const Eigen::Matrix3d &baseRotation) {
  return pinocchio::SE3(baseRotation, com - baseRotation * com_from_waist_);
}

pinocchio::SE3 BipedIG::computeBase(const Eigen::Vector3d &com,
                                    const pinocchio::SE3 &leftFoot,
                                    const pinocchio::SE3 &rightFoot) {
  double leftYawl, rightYawl;
  leftYawl = pinocchio::log3(leftFoot.rotation())(2);
  rightYawl = pinocchio::log3(rightFoot.rotation())(2);

  return computeBase(com, Eigen::AngleAxisd((leftYawl + rightYawl) / 2,
                                            Eigen::Vector3d(0, 0, 1))
                              .toRotationMatrix());
}

void BipedIG::solve(const pinocchio::SE3 &base, const pinocchio::SE3 &leftFoot,
                    const pinocchio::SE3 &rightFoot, const Eigen::VectorXd &q0,
                    Eigen::VectorXd &posture) {
  posture = q0;
  posture.head<3>() = base.translation();
  Eigen::Quaterniond quat(base.rotation());
  quat.normalize();
  posture.segment<4>(3) << quat.x(), quat.y(), quat.z(), quat.w();

  // Get the legs joints configuration for the test
  posture.segment<6>(lleg_idx_qs_) = left_leg_.solve(base, leftFoot);
  posture.segment<6>(rleg_idx_qs_) = right_leg_.solve(base, rightFoot);
}

void BipedIG::solve(const Eigen::Isometry3d &base,
                    const Eigen::Isometry3d &leftFoot,
                    const Eigen::Isometry3d &rightFoot,
                    const Eigen::VectorXd &q0, Eigen::VectorXd &posture) {
  pinocchio::SE3 root(base.matrix());
  pinocchio::SE3 LF(leftFoot.matrix());
  pinocchio::SE3 RF(rightFoot.matrix());
  solve(root, LF, RF, q0, posture);
}

void BipedIG::solve(const Eigen::Vector3d &com, const pinocchio::SE3 &leftFoot,
                    const pinocchio::SE3 &rightFoot, const Eigen::VectorXd &q0,
                    Eigen::VectorXd &posture, const double &tolerance,
                    const int &max_iterations) {
  correctCoMfromWaist(com, leftFoot, rightFoot, q0, tolerance, max_iterations);
  pinocchio::SE3 base = computeBase(com, leftFoot, rightFoot);
  solve(base, leftFoot, rightFoot, q0, posture);
}

void BipedIG::solve(const Eigen::Vector3d &com,
                    const Eigen::Isometry3d &leftFoot,
                    const Eigen::Isometry3d &rightFoot,
                    const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
                    const double &tolerance, const int &max_iterations) {
  pinocchio::SE3 LF(leftFoot.matrix());
  pinocchio::SE3 RF(rightFoot.matrix());
  solve(com, LF, RF, q0, posture, tolerance, max_iterations);
}

void BipedIG::solve(const Eigen::Vector3d &com,
                    const Eigen::Matrix3d &baseRotation,
                    const pinocchio::SE3 &leftFoot,
                    const pinocchio::SE3 &rightFoot, const Eigen::VectorXd &q0,
                    Eigen::VectorXd &posture, const double &tolerance,
                    const int &max_iterations) {
  correctCoMfromWaist(com, leftFoot, rightFoot, q0, tolerance, max_iterations);
  pinocchio::SE3 base = computeBase(com, baseRotation);
  solve(base, leftFoot, rightFoot, q0, posture);
}

void BipedIG::solve(const Eigen::Vector3d &com,
                    const Eigen::Matrix3d &baseRotation,
                    const Eigen::Isometry3d &leftFoot,
                    const Eigen::Isometry3d &rightFoot,
                    const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
                    const double &tolerance, const int &max_iterations) {
  pinocchio::SE3 LF(leftFoot.matrix());
  pinocchio::SE3 RF(rightFoot.matrix());
  solve(com, baseRotation, LF, RF, q0, posture, tolerance, max_iterations);
}

void BipedIG::derivatives(const Eigen::VectorXd &q1, const Eigen::VectorXd &q3,
                          Eigen::VectorXd &posture, Eigen::VectorXd &velocity,
                          Eigen::VectorXd &acceleration, const double &dt) {
  Eigen::VectorXd velocity1(pinocchio::difference(model_, q1, posture) / dt);
  Eigen::VectorXd velocity3(pinocchio::difference(model_, posture, q3) / dt);

  velocity = pinocchio::difference(model_, q1, q3) / (2 * dt);
  acceleration = (velocity3 - velocity1) / dt;
}

void BipedIG::solve(const std::array<pinocchio::SE3, 3> &bases,
                    const std::array<pinocchio::SE3, 3> &leftFeet,
                    const std::array<pinocchio::SE3, 3> &rightFeet,
                    const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
                    Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration,
                    const double &dt) {
  Eigen::VectorXd q1, q3;
  solve(bases[0], leftFeet[0], rightFeet[0], q0, q1);
  solve(bases[1], leftFeet[1], rightFeet[1], q0, posture);
  solve(bases[2], leftFeet[2], rightFeet[2], q0, q3);

  derivatives(q1, q3, posture, velocity, acceleration, dt);
}

void BipedIG::solve(const std::array<Eigen::Isometry3d, 3> &bases,
                    const std::array<Eigen::Isometry3d, 3> &leftFeet,
                    const std::array<Eigen::Isometry3d, 3> &rightFeet,
                    const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
                    Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration,
                    const double &dt) {
  Eigen::VectorXd q1, q3;
  solve(bases[0], leftFeet[0], rightFeet[0], q0, q1);
  solve(bases[1], leftFeet[1], rightFeet[1], q0, posture);
  solve(bases[2], leftFeet[2], rightFeet[2], q0, q3);

  derivatives(q1, q3, posture, velocity, acceleration, dt);
}

void BipedIG::solve(const std::array<Eigen::Vector3d, 3> &coms,
                    const std::array<pinocchio::SE3, 3> &leftFeet,
                    const std::array<pinocchio::SE3, 3> &rightFeet,
                    const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
                    Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration,
                    const double &dt, const double &tolerance,
                    const int &max_iterations) {
  Eigen::VectorXd q1, q3;
  solve(coms[0], leftFeet[0], rightFeet[0], q0, q1, tolerance, max_iterations);
  solve(coms[1], leftFeet[1], rightFeet[1], q0, posture, tolerance,
        max_iterations);
  solve(coms[2], leftFeet[2], rightFeet[2], q0, q3, tolerance, max_iterations);

  derivatives(q1, q3, posture, velocity, acceleration, dt);
}

void BipedIG::solve(const std::array<Eigen::Vector3d, 3> &coms,
                    const std::array<Eigen::Isometry3d, 3> &leftFeet,
                    const std::array<Eigen::Isometry3d, 3> &rightFeet,
                    const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
                    Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration,
                    const double &dt, const double &tolerance,
                    const int &max_iterations) {
  Eigen::VectorXd q1, q3;
  solve(coms[0], leftFeet[0], rightFeet[0], q0, q1, tolerance, max_iterations);
  solve(coms[1], leftFeet[1], rightFeet[1], q0, posture, tolerance,
        max_iterations);
  solve(coms[2], leftFeet[2], rightFeet[2], q0, q3, tolerance, max_iterations);

  derivatives(q1, q3, posture, velocity, acceleration, dt);
}  // @TODO: Include the parameter tolerance in each method solve. and
   // incorporate the correctCoMfromWaist in the methods solve.

void BipedIG::solve(const std::array<Eigen::Vector3d, 3> &coms,
                    const std::array<Eigen::Matrix3d, 3> &baseRotations,
                    const std::array<pinocchio::SE3, 3> &leftFeet,
                    const std::array<pinocchio::SE3, 3> &rightFeet,
                    const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
                    Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration,
                    const double &dt, const double &tolerance,
                    const int &max_iterations) {
  Eigen::VectorXd q1, q3;
  solve(coms[0], baseRotations[0], leftFeet[0], rightFeet[0], q0, q1, tolerance,
        max_iterations);
  solve(coms[1], baseRotations[1], leftFeet[1], rightFeet[1], q0, posture,
        tolerance, max_iterations);
  solve(coms[2], baseRotations[2], leftFeet[2], rightFeet[2], q0, q3, tolerance,
        max_iterations);

  derivatives(q1, q3, posture, velocity, acceleration, dt);
}

void BipedIG::solve(const std::array<Eigen::Vector3d, 3> &coms,
                    const std::array<Eigen::Matrix3d, 3> &baseRotations,
                    const std::array<Eigen::Isometry3d, 3> &leftFeet,
                    const std::array<Eigen::Isometry3d, 3> &rightFeet,
                    const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
                    Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration,
                    const double &dt, const double &tolerance,
                    const int &max_iterations) {
  Eigen::VectorXd q1, q3;
  solve(coms[0], baseRotations[0], leftFeet[0], rightFeet[0], q0, q1, tolerance,
        max_iterations);
  solve(coms[1], baseRotations[1], leftFeet[1], rightFeet[1], q0, posture,
        tolerance, max_iterations);
  solve(coms[2], baseRotations[2], leftFeet[2], rightFeet[2], q0, q3, tolerance,
        max_iterations);

  derivatives(q1, q3, posture, velocity, acceleration, dt);
}

void BipedIG::set_com_from_waist(const Eigen::Vector3d &com_from_waist) {
  com_from_waist_ = com_from_waist;
}

void BipedIG::set_com_from_waist(const Eigen::VectorXd &q) {
  Eigen::Quaterniond baseRotation(q(6), q(3), q(4), q(5));
  Eigen::Vector3d com_from_waist =
      baseRotation.matrix().transpose() *
      (pinocchio::centerOfMass(model_, data_, q) - q.head(3));
  set_com_from_waist(com_from_waist);
}

void BipedIG::correctCoMfromWaist(const Eigen::Vector3d &com,
                                  const pinocchio::SE3 &leftFoot,
                                  const pinocchio::SE3 &rightFoot,
                                  const Eigen::VectorXd &q0,
                                  const double &tolerance,
                                  const int &max_iterations) {
  error_ << 1, 1, 1;
  baseRotation_temp_ = computeBase(com, leftFoot, rightFoot).rotation();
  int i = 0;
  while (error_.norm() > tolerance && i++ < max_iterations) {
    solve(com, leftFoot, rightFoot, q0, posture_temp_);
    com_temp_ = pinocchio::centerOfMass(model_, data_, posture_temp_);
    error_ = com_temp_ - com;
    com_from_waist_ += baseRotation_temp_.transpose() * (1.2 * error_);
  }
}
// @TODO: Use this function to initialize the posture reference
// @TODO: after some iterations, it converges geometrically. So, we can write
// the exact value from the convergence. by doing that, we can reduce the
// computation time and reduce the error. Or try An inner approximation.

void BipedIG::correctCoMfromWaist(const Eigen::Vector3d &com,
                                  const Eigen::Isometry3d &leftFoot,
                                  const Eigen::Isometry3d &rightFoot,
                                  const Eigen::VectorXd &q0,
                                  const double &tolerance,
                                  const int &max_iterations) {
  pinocchio::SE3 LF(leftFoot.matrix());
  pinocchio::SE3 RF(rightFoot.matrix());
  correctCoMfromWaist(com, LF, RF, q0, tolerance, max_iterations);
}

// DYNAMICS

void BipedIG::computeDynamics(const Eigen::VectorXd &posture,
                              const Eigen::VectorXd &velocity,
                              const Eigen::VectorXd &acceleration,
                              const Eigen::Matrix<double, 6, 1> &externalWrench,
                              bool flatHorizontalGround) {

  dynamics_.computeDynamics(posture, velocity, acceleration, externalWrench, flatHorizontalGround);
  acom_ = dynamics_.getACoM();
  dL_ = dynamics_.getAMVariation();
  L_ = dynamics_.getAM();
  groundForce_ = dynamics_.getGroundCoMForce();
  groundCoMTorque_ = dynamics_.getGroundCoMTorque();
  cop_ = dynamics_.getCoP();
}

void BipedIG::computeNL(const double &w, const Eigen::VectorXd &posture,
                        const Eigen::VectorXd &velocity,
                        const Eigen::VectorXd &acceleration,
                        const Eigen::Matrix<double, 6, 1> &externalWrench,
                        bool flatHorizontalGround) {
  dynamics_.computeNL(w, posture, velocity, acceleration, externalWrench, flatHorizontalGround);
  n_ = dynamics_.getNL();
}

void BipedIG::computeNL(const double &w) {
  /**
   * In this function form, computeDynamics is suposed to have been called
   * before.
   */
  dynamics_.computeNL(w);
  n_ = dynamics_.getNL();
}

}  // namespace aig
