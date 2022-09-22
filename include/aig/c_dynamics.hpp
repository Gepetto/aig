/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Class to perform inverse geometry on a biped robot.
 */

#ifndef CENTROIDAL_DYNAMICS
#define CENTROIDAL_DYNAMICS

// clang-format off
#include <Eigen/Dense>
#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/spatial/se3.hpp"
#include <map>
#include <memory>
// clang-format on

namespace aig {

struct Contact6DSettings {
 public:
  double mu, gu;
  double half_length, half_width;
  bool active;
  Eigen::Matrix<double, 6, 1> weights;
  std::string frame_name;

  friend std::ostream &operator<<(std::ostream &out,
                                  const Contact6DSettings &obj) {
    out << "Contact6D "
        << ":\n";
    out << "    mu: " << obj.active << "\n";
    out << "    mu: " << obj.mu << "\n";
    out << "    gu: " << obj.gu << "\n";
    out << "    weights: " << obj.weights << "\n";
    out << "    Surface half_length: " << obj.half_length << "\n";
    out << "    Surface half_width: " << obj.half_width << std::endl;

    return out;
  }

  friend bool operator==(const Contact6DSettings &lhs,
                         const Contact6DSettings &rhs) {
    bool test = true;
    test &= lhs.frame_name == rhs.frame_name;
    test &= lhs.mu == rhs.mu;
    test &= lhs.active == rhs.active;
    test &= lhs.gu == rhs.gu;
    test &= lhs.half_length == rhs.half_length;
    test &= lhs.half_width == rhs.half_width;
    test &= lhs.weights == rhs.weights;
    return test;
  }
};

class Contact6D {
 private:
  Contact6DSettings settings_;
  Eigen::Matrix<double, 6, 6> oAf_;

  // matrices
  Eigen::Matrix<double, 5, 6> unilaterality_A_;
  Eigen::Matrix<double, 5, 1> unilaterality_b_;
  Eigen::Matrix<double, 6, 6> friction_A_;
  Eigen::Matrix<double, 6, 1> friction_b_;
  Eigen::Matrix<double, 6, 1> regularization_A_;
  Eigen::Matrix<double, 6, 1> regularization_b_;
  Eigen::Matrix<double, 6, 6> newton_euler_A_;
  size_t frameID_;
  Eigen::Matrix<double, 6, 1> contactForce_;

 public:
  Contact6D();
  Contact6D(const Contact6DSettings &settings);
  void initialize(const Contact6DSettings &settings);

  ~Contact6D();

  // setters
  void active(const bool &active);
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

  // getters
  const Contact6DSettings &getSettings() { return settings_; }
  const Eigen::Matrix<double, 6, 6> &toWorldForces() { return oAf_; }
  size_t uni_rows() const { return unilaterality_A_.rows(); }
  size_t fri_rows() const { return friction_A_.rows(); }
  size_t cols() const { return newton_euler_A_.cols(); }
  size_t getFrameID() const { return frameID_; }

  const Eigen::Matrix<double, 5, 6> &uni_A() { return unilaterality_A_; }
  const Eigen::Matrix<double, 5, 1> &uni_b() { return unilaterality_b_; }
  const Eigen::Matrix<double, 6, 6> &fri_A() { return friction_A_; }
  const Eigen::Matrix<double, 6, 1> &fri_b() { return friction_b_; }
  const Eigen::Matrix<double, 6, 1> &reg_A() { return regularization_A_; }
  const Eigen::Matrix<double, 6, 1> &reg_b() { return regularization_b_; }
  const Eigen::Matrix<double, 6, 6> &NE_A() { return newton_euler_A_; }

  const Eigen::Matrix<double, 6, 1> &appliedForce() { return contactForce_; }
};

struct DynoSettings {
 public:
  /**
   * @brief This must contain either a valid path to the urdf file or the
   * content of this file in a string.
   */
  std::string urdf = "";

  friend std::ostream &operator<<(std::ostream &out, const DynoSettings &obj) {
    out << "DynoSettings:\n";
    out << "    urdf: " << obj.urdf << std::endl;
    return out;
  }

  friend bool operator==(const DynoSettings &lhs, const DynoSettings &rhs) {
    bool test = true;
    test &= lhs.urdf == rhs.urdf;
    return test;
  }
};

class Dyno {
 private:
  DynoSettings settings_;
  Eigen::Matrix2d S_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  double mass_;

  // Lists of contacts
  // std::vector< std::shared_ptr<Contact6D> > known_contact6ds_;
  std::map<std::string, std::shared_ptr<Contact6D> > known_contact6ds_;
  std::vector<std::string> active_contact6ds_;

  // QP Matrices
  Eigen::MatrixXd unilaterality_A_;
  Eigen::VectorXd unilaterality_b_;
  Eigen::MatrixXd friction_A_;
  Eigen::VectorXd friction_b_;
  Eigen::VectorXd regularization_A_;
  Eigen::VectorXd regularization_b_;
  Eigen::Matrix<double, 6, -1> newton_euler_A_;
  Eigen::Matrix<double, 6, 1> newton_euler_b_;

  Eigen::MatrixXd G_, CI_, CE_;
  Eigen::VectorXd g0_, ci0_, ce0_;
  Eigen::VectorXd F_;
  Eigen::VectorXi ActiveSet_;
  int activeSetSize_;
  // Eigen::QuadProgStatus status;

  size_t cols_, uni_rows_, fri_rows_;
  // active sizes:
  size_t uni_i_, fri_i_, j_;

  // Internal variables:
  Eigen::Vector3d groundCoMForce_, groundCoMTorque_, nonCoPTorque_, weight_;
  Eigen::Vector3d acom_;
  Eigen::Vector2d cop_;
  Eigen::Vector3d dL_;
  Eigen::Vector3d L_;
  Eigen::Vector2d n_;

  void addSizes(const std::shared_ptr<Contact6D> &contact);
  void removeSizes(const std::shared_ptr<Contact6D> &contact);
  void resizeMatrices();
  void buildMatrices(const Eigen::Vector3d &groundCoMForce,
                     const Eigen::Vector3d &groundCoMTorque,
                     const Eigen::Vector3d &CoM);
  void solveQP();
  void distribute();

 public:
  Dyno();
  Dyno(const DynoSettings settings);
  void initialize(const DynoSettings settings);

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

  void addContact6d(const std::shared_ptr<Contact6D> &contact,
                    const std::string &name);
  void removeContact6d(const std::string &name);

  void activateContact6d(const std::string &name);
  void deactivateContact6d(const std::string &name);

  void distributeForce(const Eigen::Vector3d &groundCoMForce,
                       const Eigen::Vector3d &groundCoMTorque,
                       const Eigen::Vector3d &CoM);

  // GETTERS
  /// @brief Please call computeDynamics first.
  const Eigen::Vector3d &getAMVariation() { return dL_; }
  const Eigen::Vector3d &getCoM() { return data_.com[0]; }
  const Eigen::Vector3d &getVCoM() { return data_.vcom[0]; }
  const Eigen::Vector3d &getACoM() { return acom_; }
  const Eigen::Vector3d &getAM() { return L_; }
  const Eigen::Vector2d &getCoP() { return cop_; }
  const Eigen::Vector2d &getNL() { return n_; }
  const Eigen::Vector3d &getGroundCoMForce() { return groundCoMForce_; }
  const Eigen::Vector3d &getGroundCoMTorque() { return groundCoMTorque_; }
  const std::vector<std::string> &getActiveContacts() {
    return active_contact6ds_;
  }
  const std::shared_ptr<Contact6D> &getContact(std::string name) {
    return known_contact6ds_[name];
  }
};

}  // namespace aig

#endif  // CENTROIDAL_DYNAMICS
