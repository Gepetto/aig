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
#include "aig/dyna_com.hpp"
// clang-format on

namespace aig {
/**
 * @brief @todo Describe BipedIGSettings
 */

struct BipedIGSettings {
public:
  std::string left_hip_joint_name;
  std::string left_knee_joint_name;
  std::string left_ankle_joint_name;
  std::string left_foot_frame_name;
  std::string right_hip_joint_name;
  std::string right_knee_joint_name;
  std::string right_ankle_joint_name;
  std::string right_foot_frame_name;
  /**
   * @brief This must contain either a valid path to the urdf file or the
   * content of this file in a string.
   */
  std::string urdf;
  /**
   * @brief This must contain either a valid path to the srdf file or the
   * content of this file in a string.
   */
  std::string srdf;

  BipedIGSettings()
      : left_hip_joint_name(""), left_knee_joint_name(""),
        left_ankle_joint_name(""), left_foot_frame_name(""),
        right_hip_joint_name(""), right_knee_joint_name(""),
        right_ankle_joint_name(""), right_foot_frame_name(""), urdf(""),
        srdf("") {}

  BipedIGSettings(const std::string &_left_hip_joint_name,
                  const std::string &_left_knee_joint_name,
                  const std::string &_left_ankle_joint_name,
                  const std::string &_left_foot_frame_name,
                  const std::string &_right_hip_joint_name,
                  const std::string &_right_knee_joint_name,
                  const std::string &_right_ankle_joint_name,
                  const std::string &_right_foot_frame_name,
                  const std::string &_urdf, const std::string &_srdf)
      : left_hip_joint_name(_left_hip_joint_name),
        left_knee_joint_name(_left_knee_joint_name),
        left_ankle_joint_name(_left_ankle_joint_name),
        left_foot_frame_name(_left_foot_frame_name),
        right_hip_joint_name(_right_hip_joint_name),
        right_knee_joint_name(_right_knee_joint_name),
        right_ankle_joint_name(_right_ankle_joint_name),
        right_foot_frame_name(_right_foot_frame_name), urdf(_urdf),
        srdf(_srdf) {}

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
    out << "    urdf: " << obj.urdf << "\n";
    out << "    srdf: " << obj.srdf << std::endl;
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
    test &= lhs.urdf == rhs.urdf;
    test &= lhs.srdf == rhs.srdf;
    return test;
  }
};

BipedIGSettings makeSettingsFor(const std::string &path_to_robots,
                                const std::string &robot_name);

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
  // ArmIG left_arm_, right_arm_;
  Eigen::VectorXd q0_; // q0_ is a reference configuration used to take all not
                       // computed joints (such as head and arms)
  Eigen::Vector3d com_from_waist_;
  int lleg_idx_qs_; // Indexes in the configuration vector.
  int rleg_idx_qs_; // Indexes in the configuration vector.

  // variables used in the waist-com vector correction:
  Eigen::Vector3d error_, com_temp_;
  Eigen::VectorXd posture_temp_;
  Eigen::Matrix3d baseRotation_temp_;

  aig::DynaCoM dynamics_;

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

  void configureLegs();

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
  void setQ0(const Eigen::VectorXd q0) { q0_ = q0; }

  /// @brief Get the Angular Momentum variation. Please call computeDynamics
  /// first. Deprecate it, AIG is not made for dynamics
  const Eigen::Vector3d &getAMVariation() { return dynamics_.getAMVariation(); }

  /// @brief Get the CoM position. Please call computeDynamics first.
  const Eigen::Vector3d &getCoM() { return dynamics_.getCoM(); }
  /// @brief Get the CoM velocity. Please call computeDynamics first.
  const Eigen::Vector3d &getVCoM() { return dynamics_.getVCoM(); }
  /// @brief Get the CoM acceleration. Please call computeDynamics first.
  const Eigen::Vector3d &getACoM() { return dynamics_.getACoM(); }
  /// @brief Get the angular momentum. Please call computeDynamics first.
  const Eigen::Vector3d &getAM() { return dynamics_.getAM(); }
  /// @brief Get the CoP Position. Please call computeDynamics first.
  const Eigen::Vector2d &getCoP() { return dynamics_.getCoP(); }
  /// @brief Get the nonlinear effect. Please call computeDynamics first.
  const Eigen::Vector2d &getNL() { return dynamics_.getNL(); }

  void checkCompatibility(); // TODO

  void solve(const Eigen::Vector3d &com, const pinocchio::SE3 &leftFoot,
             const pinocchio::SE3 &rightFoot, const Eigen::VectorXd &q0,
             Eigen::VectorXd &posture, const double &tolerance = 1e-10,
             const int &max_iterations = 0);

  void solve(const Eigen::Vector3d &com, const Eigen::Isometry3d &leftFeet,
             const Eigen::Isometry3d &rightFeet, const Eigen::VectorXd &q0,
             Eigen::VectorXd &posture, const double &tolerance = 1e-10,
             const int &max_iterations = 0);

  void solve(const Eigen::Vector3d &com, const Eigen::Matrix3d &baseRotation,
             const pinocchio::SE3 &leftFoot, const pinocchio::SE3 &rightFoot,
             const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
             const double &tolerance = 1e-10, const int &max_iterations = 0);

  void solve(const Eigen::Vector3d &com, const Eigen::Matrix3d &baseRotation,
             const Eigen::Isometry3d &leftFoot,
             const Eigen::Isometry3d &rightFoot, const Eigen::VectorXd &q0,
             Eigen::VectorXd &posture, const double &tolerance = 1e-10,
             const int &max_iterations = 0);

  void solve(const pinocchio::SE3 &base, const pinocchio::SE3 &leftFoot,
             const pinocchio::SE3 &rightFoot, const Eigen::VectorXd &q0,
             Eigen::VectorXd &posture);

  void solve(const Eigen::Isometry3d &base, const Eigen::Isometry3d &leftFoot,
             const Eigen::Isometry3d &rightFoot, const Eigen::VectorXd &q0,
             Eigen::VectorXd &posture);

  void solve(const std::array<Eigen::Vector3d, 3> &coms,
             const std::array<pinocchio::SE3, 3> &leftFeet,
             const std::array<pinocchio::SE3, 3> &rightFeet,
             const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
             Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration,
             const double &dt, const double &tolerance = 1e-10,
             const int &max_iterations = 0);

  void solve(const std::array<Eigen::Vector3d, 3> &coms,
             const std::array<Eigen::Isometry3d, 3> &leftFeet,
             const std::array<Eigen::Isometry3d, 3> &rightFeet,
             const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
             Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration,
             const double &dt, const double &tolerance = 1e-10,
             const int &max_iterations = 0);

  void solve(const std::array<Eigen::Vector3d, 3> &coms,
             const std::array<Eigen::Matrix3d, 3> &baseRotations,
             const std::array<pinocchio::SE3, 3> &leftFeet,
             const std::array<pinocchio::SE3, 3> &rightFeet,
             const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
             Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration,
             const double &dt, const double &tolerance = 1e-10,
             const int &max_iterations = 0);

  void solve(const std::array<Eigen::Vector3d, 3> &coms,
             const std::array<Eigen::Matrix3d, 3> &baseRotations,
             const std::array<Eigen::Isometry3d, 3> &leftFeet,
             const std::array<Eigen::Isometry3d, 3> &rightFeet,
             const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
             Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration,
             const double &dt, const double &tolerance = 1e-10,
             const int &max_iterations = 0);

  void solve(const std::array<pinocchio::SE3, 3> &bases,
             const std::array<pinocchio::SE3, 3> &leftFeet,
             const std::array<pinocchio::SE3, 3> &rightFeet,
             const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
             Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration,
             const double &dt);

  void solve(const std::array<Eigen::Isometry3d, 3> &bases,
             const std::array<Eigen::Isometry3d, 3> &leftFeet,
             const std::array<Eigen::Isometry3d, 3> &rightFeet,
             const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
             Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration,
             const double &dt);

  void set_com_from_waist(const Eigen::Vector3d &com_from_waist);

  void set_com_from_waist(const Eigen::VectorXd &q);

  void correctCoMfromWaist(const Eigen::Vector3d &com,
                           const pinocchio::SE3 &leftFoot,
                           const pinocchio::SE3 &rightFoot,
                           const Eigen::VectorXd &q0,
                           const double &tolerance = 1e-10,
                           const int &max_iterations = 20);

  void correctCoMfromWaist(const Eigen::Vector3d &com,
                           const Eigen::Isometry3d &leftFoot,
                           const Eigen::Isometry3d &rightFoot,
                           const Eigen::VectorXd &q0,
                           const double &tolerance = 1e-10,
                           const int &max_iterations = 20);

  void computeDynamics(const Eigen::VectorXd &posture,
                       const Eigen::VectorXd &velocity,
                       const Eigen::VectorXd &acceleration,
                       const Eigen::Matrix<double, 6, 1> &externalWrench =
                           Eigen::Matrix<double, 6, 1>::Zero(),
                       bool flatHorizontalGround = true);

  void computeNL(const double &w, const Eigen::VectorXd &posture,
                 const Eigen::VectorXd &velocity,
                 const Eigen::VectorXd &acceleration,
                 const Eigen::Matrix<double, 6, 1> &externalWrench =
                     Eigen::Matrix<double, 6, 1>::Zero(),
                 bool flatHorizontalGround = true);

  void computeNL(const double &w);

  pinocchio::Model &get_model() { return model_; }
  pinocchio::Data &get_data() { return data_; }
  Eigen::Vector3d &get_com_from_waist() { return com_from_waist_; }
};
} // namespace aig
#endif // AIG_BIPED_IG
