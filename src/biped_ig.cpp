/**
 * @file
 * @copyright Copyright (c) 2022, LAAS-CNRS, Toward, PalRobotics
 * @brief
 */

#include "preview_ik/biped_ig.hpp"

#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace preview_ik {

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

  // Build the robot model and cache.
  pinocchio::urdf::buildModel(settings_.urdf_path,
                              pinocchio::JointModelFreeFlyer(), model_);
  data_ = pinocchio::Data(model_);

  // Extract the CoM to Waist level arm.
  pinocchio::srdf::loadReferenceConfigurations(model_, settings_.srdf_path,
                                               false);
  q0_ = model_.referenceConfigurations["half_sitting"];
  pinocchio::Data data(model_);
  com_from_waist_ = pinocchio::centerOfMass(model_, data_, q0_) - q0_.head(3);

  configurateLegs();
}

void BipedIG::configurateLegs() {
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
                            model_.getFrameId(settings_.left_foot_frame_name),
                        rightSoleID =
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
}

pinocchio::SE3 BipedIG::computeBase(const Eigen::Vector3d &com,
                                    const Eigen::Matrix3d &baseRotation) {
  pinocchio::SE3 base;
  base.rotation(baseRotation);
  base.translation(com - base.rotation() * com_from_waist_);
  return base;
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
  posture.head(3) = base.translation();
  Eigen::Quaterniond quat(base.rotation());
  quat.normalize();
  posture.segment(3, 4) << quat.x(), quat.y(), quat.z(), quat.w();
  posture.segment(7, 6) = left_leg_.solve(base, leftFoot);
  posture.segment(13, 6) = right_leg_.solve(base, rightFoot);
  // TODO the numbers 7, 6, 13, 6 could be not hard coded.
}

void BipedIG::solve(const Eigen::Vector3d &com, const pinocchio::SE3 &leftFoot,
                    const pinocchio::SE3 &rightFoot, const Eigen::VectorXd &q0,
                    Eigen::VectorXd &posture) {
  pinocchio::SE3 base = computeBase(com, leftFoot, rightFoot);
  solve(base, leftFoot, rightFoot, q0, posture);
}

void BipedIG::solve(const Eigen::Vector3d &com,
                    const Eigen::Matrix3d &baseRotation,
                    const pinocchio::SE3 &leftFoot,
                    const pinocchio::SE3 &rightFoot, const Eigen::VectorXd &q0,
                    Eigen::VectorXd &posture) {
  pinocchio::SE3 base = computeBase(com, baseRotation);
  solve(base, leftFoot, rightFoot, q0, posture);
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

void BipedIG::solve(const std::array<Eigen::Vector3d, 3> &coms,
                    const std::array<pinocchio::SE3, 3> &leftFeet,
                    const std::array<pinocchio::SE3, 3> &rightFeet,
                    const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
                    Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration,
                    const double &dt) {
  Eigen::VectorXd q1, q3;
  solve(coms[0], leftFeet[0], rightFeet[0], q0, q1);
  solve(coms[1], leftFeet[1], rightFeet[1], q0, posture);
  solve(coms[2], leftFeet[2], rightFeet[2], q0, q3);

  derivatives(q1, q3, posture, velocity, acceleration, dt);
}

void BipedIG::solve(const std::array<Eigen::Vector3d, 3> &coms,
                    const std::array<Eigen::Matrix3d, 3> &baseRotations,
                    const std::array<pinocchio::SE3, 3> &leftFeet,
                    const std::array<pinocchio::SE3, 3> &rightFeet,
                    const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
                    Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration,
                    const double &dt) {
  Eigen::VectorXd q1, q3;
  solve(coms[0], baseRotations[0], leftFeet[0], rightFeet[0], q0, q1);
  solve(coms[1], baseRotations[1], leftFeet[1], rightFeet[1], q0, posture);
  solve(coms[2], baseRotations[2], leftFeet[2], rightFeet[2], q0, q3);

  derivatives(q1, q3, posture, velocity, acceleration, dt);
}

Eigen::Vector2d BipedIG::computeCoP(
    const Eigen::VectorXd &posture, const Eigen::VectorXd &velocity,
    const Eigen::VectorXd &acceleration,
    const Eigen::Matrix<double, 6, 1> &externalWrench,
    bool flatHorizontalGround) {
  // The external wrench is supposed to be expressed in the frame of the root
  // link.

  Eigen::Matrix<double, 6, 1> tauMw =
      pinocchio::rnea(model_, data_, posture, velocity, acceleration).head(6);
  Eigen::Vector3d groundTorqueMo =
      tauMw.tail(3) - externalWrench.tail(3) +
      pinocchio::skew(Eigen::Vector3d(posture.head(3))) *
          (tauMw.head(3) - externalWrench.head(3));

  Eigen::Vector3d pressureTorqueMo;
  if (flatHorizontalGround) {
    pressureTorqueMo = groundTorqueMo;
  } else {
    // TODO get the force distribution and remove the non pressure terms form
    // the CoP computation. for now, we assume a flat and horizontal ground.
  }

  return Eigen::Vector2d(-pressureTorqueMo(1) / (tauMw(2) - externalWrench(2)),
                         pressureTorqueMo(0) / (tauMw(2) - externalWrench(2)));
}

Eigen::Vector2d BipedIG::computeCoP(const Eigen::VectorXd &posture,
                                    const Eigen::VectorXd &velocity,
                                    const Eigen::VectorXd &acceleration,
                                    bool flatHorizontalGround) {
  Eigen::Matrix<double, 6, 1> externalWrench =
      Eigen::Matrix<double, 6, 1>::Zero();
  return computeCoP(posture, velocity, acceleration, externalWrench,
                    flatHorizontalGround);
}

}  // namespace preview_ik