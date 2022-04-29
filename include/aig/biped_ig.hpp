/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Class to perform inverse geometry on a biped robot.
 */

#ifndef AIG_BIPED_IG
#define AIG_BIPED_IG

// clang-format off
#include <Eigen/Dense>
#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "aig/arm_ig.hpp"
#include "aig/leg_ig.hpp"
// clang-format on

namespace aig {
/**
 * @brief @todo Describe BipedIGSettings
 */

struct BipedIGSettings {
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

  friend std::ostream &operator<<(std::ostream &out,
                                  const BipedIGSettings &obj) {
    out << "BipedIGSettings:\n";
    out << "    left_hip_joint_name: " << obj.left_hip_joint_name << "\n";
    out << "    left_knee_joint_name: " << obj.left_knee_joint_name << "\n";
    out << "    left_ankle_joint_name: " << obj.left_ankle_joint_name << "\n";
    out << "    left_foot_frame_name: " << obj.left_foot_frame_name << "\n";
    out << "    right_hip_joint_name: " << obj.right_hip_joint_name << "\n";
    out << "    right_knee_joint_name: " << obj.right_knee_joint_name << "\n";
    out << "    right_ankle_joint_name: " << obj.right_ankle_joint_name << "\n";
    out << "    right_foot_frame_name: " << obj.right_foot_frame_name << "\n";
    out << "    urdf_path: " << obj.urdf_path << "\n";
    out << "    srdf_path: " << obj.srdf_path << std::endl;
    return out;
  }

  friend bool operator==(const BipedIGSettings &lhs,
                         const BipedIGSettings &rhs) {
    bool test = true;
    test &= lhs.left_hip_joint_name == rhs.left_hip_joint_name;
    test &= lhs.left_knee_joint_name == rhs.left_knee_joint_name;
    test &= lhs.left_ankle_joint_name == rhs.left_ankle_joint_name;
    test &= lhs.left_foot_frame_name == rhs.left_foot_frame_name;
    test &= lhs.right_hip_joint_name == rhs.right_hip_joint_name;
    test &= lhs.right_knee_joint_name == rhs.right_knee_joint_name;
    test &= lhs.right_ankle_joint_name == rhs.right_ankle_joint_name;
    test &= lhs.right_foot_frame_name == rhs.right_foot_frame_name;
    test &= lhs.urdf_path == rhs.urdf_path;
    test &= lhs.srdf_path == rhs.srdf_path;
    return test;
  }
};

BipedIGSettings makeSettingsFor(std::string robot_name);


/**
 * @brief @todo Describe BipedIG
 *
 */
class BipedIG {
  // Private attributes.
 private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  BipedIGSettings settings_;
  LegIG left_leg_, right_leg_;
  //ArmIG left_arm_, right_arm_;
  Eigen::VectorXd q0_;
  Eigen::Vector3d com_from_waist_;
  int lleg_idx_qs_;  // Indexes in the configuration vector.
  int rleg_idx_qs_;  // Indexes in the configuration vector.
  double mass_;
  double gravity_;
  Eigen::Vector2d cop_;
  Eigen::Vector3d dL_;
  Eigen::Vector2d n_;

  // Private methods.
 private:
  void derivatives(const Eigen::VectorXd &q1, const Eigen::VectorXd &q3,
                   Eigen::VectorXd &posture, Eigen::VectorXd &velocity,
                   Eigen::VectorXd &acceleration, const double &dt);

  pinocchio::SE3 computeBase(const Eigen::Vector3d &com,
                             const pinocchio::SE3 &leftFoot,
                             const pinocchio::SE3 &rightFoot);

  pinocchio::SE3 computeBase(const Eigen::Vector3d &com,
                             const Eigen::Matrix3d &baseRotation);

  void configurateLegs();

  // Public methods.
 public:
  BipedIG();

  BipedIG(const BipedIGSettings &settings);

  void initialize(const BipedIGSettings &settings);

  const BipedIGSettings &get_settings() { return settings_; };
  const LegIGSettings &get_left_leg_settings() {
    return left_leg_.get_settings();
  };
  const LegIGSettings &get_right_leg_settings() {
    return right_leg_.get_settings();
  };

  const Eigen::VectorXd &getQ0() { return q0_; }

  const Eigen::Vector3d &getAMVariation() { return dL_; }
  const Eigen::Vector2d &getCoP() { return cop_; }
  const Eigen::Vector2d &getNL() { return n_; }

  void checkCompatibility();  // TODO

  void solve(const Eigen::Vector3d &com, 
             const pinocchio::SE3 &leftFoot,
             const pinocchio::SE3 &rightFoot, 
             const Eigen::VectorXd &q0,
             Eigen::VectorXd &posture);

  void solve(const Eigen::Vector3d &com,
             const Eigen::Isometry3d &leftFeet,
             const Eigen::Isometry3d &rightFeet,
             const Eigen::VectorXd &q0, 
             Eigen::VectorXd &posture);

  void solve(const Eigen::Vector3d &com, 
             const Eigen::Matrix3d &baseRotation,
             const pinocchio::SE3 &leftFoot, 
             const pinocchio::SE3 &rightFoot,
             const Eigen::VectorXd &q0, 
             Eigen::VectorXd &posture);

  void solve(const Eigen::Vector3d &com,
             const Eigen::Matrix3d &baseRotation,
             const Eigen::Isometry3d &leftFoot,
             const Eigen::Isometry3d &rightFoot,
             const Eigen::VectorXd &q0, 
             Eigen::VectorXd &posture);

  void solve(const pinocchio::SE3 &base, 
             const pinocchio::SE3 &leftFoot,
             const pinocchio::SE3 &rightFoot, 
             const Eigen::VectorXd &q0,
             Eigen::VectorXd &posture);

  void solve(const Eigen::Isometry3d &base, 
             const Eigen::Isometry3d &leftFoot,
             const Eigen::Isometry3d &rightFoot,
             const Eigen::VectorXd &q0, 
             Eigen::VectorXd &posture);

  void solve(const std::array<Eigen::Vector3d, 3> &coms,
             const std::array<pinocchio::SE3, 3> &leftFeet,
             const std::array<pinocchio::SE3, 3> &rightFeet,
             const Eigen::VectorXd &q0, 
             Eigen::VectorXd &posture,
             Eigen::VectorXd &velocity, 
             Eigen::VectorXd &acceleration,
             const double &dt);
            
  void solve(const std::array<Eigen::Vector3d, 3> &coms,
             const std::array<Eigen::Isometry3d, 3> &leftFeet,
             const std::array<Eigen::Isometry3d, 3> &rightFeet,
             const Eigen::VectorXd &q0, 
             Eigen::VectorXd &posture,
             Eigen::VectorXd &velocity, 
             Eigen::VectorXd &acceleration,
             const double &dt);

  void solve(const std::array<Eigen::Vector3d, 3> &coms,
             const std::array<Eigen::Matrix3d, 3> &baseRotations,
             const std::array<pinocchio::SE3, 3> &leftFeet,
             const std::array<pinocchio::SE3, 3> &rightFeet,
             const Eigen::VectorXd &q0, 
             Eigen::VectorXd &posture,
             Eigen::VectorXd &velocity, 
             Eigen::VectorXd &acceleration,
             const double &dt);

  void solve(const std::array<Eigen::Vector3d, 3> &coms,
             const std::array<Eigen::Matrix3d, 3> &baseRotations,
             const std::array<Eigen::Isometry3d, 3> &leftFeet,
             const std::array<Eigen::Isometry3d, 3> &rightFeet,
             const Eigen::VectorXd &q0, 
             Eigen::VectorXd &posture,
             Eigen::VectorXd &velocity, 
             Eigen::VectorXd &acceleration,
             const double &dt);

  void solve(const std::array<pinocchio::SE3, 3> &bases,
             const std::array<pinocchio::SE3, 3> &leftFeet,
             const std::array<pinocchio::SE3, 3> &rightFeet,
             const Eigen::VectorXd &q0, 
             Eigen::VectorXd &posture,
             Eigen::VectorXd &velocity, 
             Eigen::VectorXd &acceleration,
             const double &dt);

  void solve(const std::array<Eigen::Isometry3d, 3> &bases,
             const std::array<Eigen::Isometry3d, 3> &leftFeet,
             const std::array<Eigen::Isometry3d, 3> &rightFeet,
             const Eigen::VectorXd &q0, 
             Eigen::VectorXd &posture,
             Eigen::VectorXd &velocity, 
             Eigen::VectorXd &acceleration,
             const double &dt);

  void set_com_from_waist(const Eigen::Vector3d &com_from_waist);

  void set_com_from_waist(const Eigen::VectorXd &q);

  void computeDynamics(const Eigen::VectorXd &posture,
                       const Eigen::VectorXd &velocity,
                       const Eigen::VectorXd &acceleration,
                       bool flatHorizontalGround = true);

  void computeDynamics(const Eigen::VectorXd &posture,
                       const Eigen::VectorXd &velocity,
                       const Eigen::VectorXd &acceleration,
                       const Eigen::Matrix<double, 6, 1> &externalWrench,
                       bool flatHorizontalGround = true);

  Eigen::Vector2d computeNL(const Eigen::VectorXd &posture, // deprecate it
                            const Eigen::VectorXd &velocity,
                            const Eigen::VectorXd &acceleration,
                            const Eigen::Matrix<double, 6, 1> &externalWrench,
                            bool flatHorizontalGround = true);

  Eigen::Vector2d computeNL(const Eigen::VectorXd &posture, // deprecate it
                            const Eigen::VectorXd &velocity,
                            const Eigen::VectorXd &acceleration,
                            bool flatHorizontalGround = true);

  pinocchio::Model &get_model() { return model_; }
};
}  // namespace aig
#endif  // AIG_BIPED_IG
