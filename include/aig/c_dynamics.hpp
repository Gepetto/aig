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

namespace dyno {

enum contact { surface, point };

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

 public:
  // matrices
  Eigen::Matrix<double, 5, 6> unilaterality_A_;
  Eigen::Matrix<double, 5, 1> unilaterality_b_;
  Eigen::Matrix<double, 6, 6> friction_A_;
  Eigen::Matrix<double, 6, 1> friction_b_;
  Eigen::DiagonalMatrix<double, 6> regularization_A_;
  Eigen::Matrix<double, 6, 1> regularization_b_;
  Eigen::Matrix<double, 6, 6> newton_euler_A_;

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
  void updateNewtonEuler(
      const Eigen::Vector3d &CoM,
      const pinocchio::SE3 &oMf);  // Check oMf, maybe we actually need fMo.

  Eigen::Matrix<double, 6, 1> contactForce;

  // getters
  const Contact6DSettings &getSettings() { return settings_; }
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
  Eigen::DiagonalMatrix<double, -1> regularization_A_;
  Eigen::VectorXd regularization_b_;
  Eigen::Matrix<double, 6, -1> newton_euler_A_;
  Eigen::Matrix<double, 6, 1> newton_euler_b_;

  size_t uni_cols_, uni_rows_, fri_cols_, fri_rows_, reg_cols_, reg_rows_,
      ne_cols_;

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
  void buildMatrices();

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
};

}  // namespace dyno

#endif  // CENTROIDAL_DYNAMICS
