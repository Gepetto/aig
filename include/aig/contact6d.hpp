/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Class to perform inverse geometry on a biped robot.
 */

#ifndef AIG_CONTACT_6D
#define AIG_CONTACT_6D

// clang-format off
#include <Eigen/Dense>
#include "pinocchio/spatial/se3.hpp"
#include <memory>
// clang-format on

namespace aig {

struct Contact6DSettings {
 public:
  double mu, gu;
  double half_length, half_width;
  Eigen::Matrix<double, 6, 1> weights;
  std::string frame_name;

  std::string to_string() {
    std::ostringstream out;
    out << "Contact6D "
        << ":\n";
    out << "    mu: " << this->mu << "\n";
    out << "    gu: " << this->gu << "\n";
    out << "    weights: " << this->weights.transpose() << "\n";
    out << "    Surface half_length: " << this->half_length << "\n";
    out << "    Surface half_width: " << this->half_width << std::endl;

    return out.str();
  }

  std::ostream &operator<<(std::ostream &out) {
    out << this->to_string();
    return out;
  }

  bool operator!=(const Contact6DSettings &rhs) { return !(*this == rhs); }

  bool operator==(const Contact6DSettings &rhs) {
    bool test = true;
    test &= this->frame_name == rhs.frame_name;
    test &= this->mu == rhs.mu;
    test &= this->gu == rhs.gu;
    test &= this->half_length == rhs.half_length;
    test &= this->half_width == rhs.half_width;
    test &= this->weights == rhs.weights;
    return test;
  }
};

class Contact6D {
 private:
  Contact6DSettings settings_;
  pinocchio::SE3 oMs_, cMo_;

  // matrices
  Eigen::Matrix<double, 5, 6> unilaterality_A_;
  Eigen::Matrix<double, 5, 1> unilaterality_b_;
  Eigen::Matrix<double, 6, 6> friction_A_;
  Eigen::Matrix<double, 6, 1> friction_b_;
  Eigen::Matrix<double, 6, 1> regularization_A_;
  Eigen::Matrix<double, 6, 1> regularization_b_;
  Eigen::Matrix<double, 6, 6> newton_euler_A_;
  Eigen::Matrix<double, 6, 1> contactForce_;
  size_t frameID_;

 public:
  Contact6D();
  Contact6D(const Contact6DSettings &settings);
  void initialize(const Contact6DSettings &settings);

  // ~Contact6D();

  // setters
  void setMu(const double &mu);
  void setGu(const double &gu);
  void setForceWeights(const Eigen::Vector3d &force_weights);
  void setTorqueWeights(const Eigen::Vector3d &torque_weights);
  void setSurfaceHalfWidth(const double &half_width);
  void setSurfaceHalfLength(const double &half_length);
  void updateNewtonEuler(const Eigen::Vector3d &CoM, const pinocchio::SE3 &oMf);
  void setFrameID(const size_t frameID) { frameID_ = frameID; }
  void applyForce(const Eigen::Matrix<double, 6, 1> &force) {
    contactForce_ << force;
  }
  void setPose(pinocchio::SE3 &pose) {oMs_ = pose; }

  // getters
  const Contact6DSettings &getSettings() { return settings_; }
  const Eigen::Matrix<double, 6, 6> toWorldForces() {
    return oMs_.toActionMatrixInverse().transpose();
  }
  const Eigen::Matrix<double, 6, 6> toCoMForces() {
    return cMo_.act(oMs_).toActionMatrixInverse().transpose();
  }
  size_t uni_rows() const { return unilaterality_A_.rows(); }
  size_t fri_rows() const { return friction_A_.rows(); }
  size_t cols() const { return newton_euler_A_.cols(); }
  size_t getFrameID() const { return frameID_; }
  const pinocchio::SE3 &getPose() const { return oMs_; }

  const Eigen::Matrix<double, 5, 6> &uni_A() { return unilaterality_A_; }
  const Eigen::Matrix<double, 5, 1> &uni_b() { return unilaterality_b_; }
  const Eigen::Matrix<double, 6, 6> &fri_A() { return friction_A_; }
  const Eigen::Matrix<double, 6, 1> &fri_b() { return friction_b_; }
  const Eigen::Matrix<double, 6, 1> &reg_A() { return regularization_A_; }
  const Eigen::Matrix<double, 6, 1> &reg_b() { return regularization_b_; }
  const Eigen::Matrix<double, 6, 6> &NE_A() { return newton_euler_A_; }

  const Eigen::Matrix<double, 6, 1> &appliedForce() { return contactForce_; }
};

///// Start of future contact point. ////////////
/**
 * Still missing the definition of the abstract class Contact which will be
 * the ansester of ContactPoint and Contact6D. Similar for the settings.
*/
struct ContactPointSettings {
 public:
  double mu;
  Eigen::Matrix<double, 3, 1> weights;
  std::string frame_name;

  std::string to_string() {
    std::ostringstream out;
    out << "Contact6D "
        << ":\n";
    out << "    mu: " << this->mu << "\n";
    out << "    weights: " << this->weights.transpose() << std::endl;

    return out.str();
  }

  std::ostream &operator<<(std::ostream &out) {
    out << this->to_string();
    return out;
  }

  bool operator==(const ContactPointSettings &rhs) {
    bool test = true;
    test &= this->frame_name == rhs.frame_name;
    test &= this->mu == rhs.mu;
    test &= this->weights == rhs.weights;
    return test;
  }

  bool operator!=(const ContactPointSettings &rhs) { return !(*this == rhs); }
};

class ContactPoint {
 private:
  ContactPointSettings settings_;
  pinocchio::SE3 oMs_, cMo_;

  // matrices
  Eigen::Matrix<double, 1, 3> unilaterality_A_;
  Eigen::Matrix<double, 1, 1> unilaterality_b_;
  Eigen::Matrix<double, 4, 3> friction_A_;
  Eigen::Matrix<double, 4, 1> friction_b_;
  Eigen::Matrix<double, 3, 1> regularization_A_;
  Eigen::Matrix<double, 3, 1> regularization_b_;
  Eigen::Matrix<double, 6, 3> newton_euler_A_;
  size_t frameID_;
  Eigen::Matrix<double, 3, 1> contactForce_;

 public:
  ContactPoint();
  ContactPoint(const ContactPointSettings &settings);
  void initialize(const ContactPointSettings &settings);

  // ~ContactPoint();

  // setters
  void setMu(const double &mu);
  void setForceWeights(const Eigen::Vector3d &force_weights);
  void updateNewtonEuler(const Eigen::Vector3d &CoM, const pinocchio::SE3 &oMf);
  void setFrameID(const size_t frameID) { frameID_ = frameID; }
  void applyForce(const Eigen::Matrix<double, 3, 1> &force) {
    contactForce_ << force;
  }

  // getters
  const ContactPointSettings &getSettings() { return settings_; }
  const Eigen::Matrix<double, 6, 3> toWorldForces() {
    return oMs_.toActionMatrixInverse().transpose().block<6, 3>(0, 0);
  }
  const Eigen::Matrix<double, 6, 3> toCoMForces() {
    return oMs_.act(cMo_).toActionMatrixInverse().transpose().block<6, 3>(0, 0);
  }
  size_t uni_rows() const { return unilaterality_A_.rows(); }
  size_t fri_rows() const { return friction_A_.rows(); }
  size_t cols() const { return newton_euler_A_.cols(); }
  size_t getFrameID() const { return frameID_; }

  const Eigen::Matrix<double, 1, 3> &uni_A() { return unilaterality_A_; }
  const Eigen::Matrix<double, 1, 1> &uni_b() { return unilaterality_b_; }
  const Eigen::Matrix<double, 4, 3> &fri_A() { return friction_A_; }
  const Eigen::Matrix<double, 4, 1> &fri_b() { return friction_b_; }
  const Eigen::Matrix<double, 3, 1> &reg_A() { return regularization_A_; }
  const Eigen::Matrix<double, 3, 1> &reg_b() { return regularization_b_; }
  const Eigen::Matrix<double, 6, 3> &NE_A() { return newton_euler_A_; }

  const Eigen::Matrix<double, 3, 1> &appliedForce() { return contactForce_; }
};

}  // namespace aig

#endif  // AIG_CONTACT_6D
