/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Class to perform inverse geometry on a biped robot.
 */

#ifndef AIG_DYNACOM
#define AIG_DYNACOM

// clang-format off
#include <Eigen/Dense>
#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/spatial/se3.hpp"
#include <map>
#include <memory>
#include "aig/contact6d.hpp"
// clang-format on

// @TODO: Separate the DynaCom in a new repository.
// @TODO: add abstract class contact and new implementation of it contactPoint.
// @TODO: implement method copy of contacts to be able to duplicate them easily.
// @TODO: In the getters uni_A, fria_A, uni_b, fri_b return only the active
// block.
// @TODO: change internaly the system of names by ids.

namespace aig {

struct DynaCoMSettings {
 public:
  /**
   * @brief This must contain either a valid path to the urdf file or the
   * content of this file in a string.
   */
  std::string urdf = "";

  friend std::ostream &operator<<(std::ostream &out,
                                  const DynaCoMSettings &obj) {
    out << "DynaCoMSettings:\n";
    out << "    urdf: " << obj.urdf << std::endl;
    return out;
  }

  friend bool operator==(const DynaCoMSettings &lhs,
                         const DynaCoMSettings &rhs) {
    bool test = true;
    test &= lhs.urdf == rhs.urdf;
    return test;
  }
};

class DynaCoM {
 private:
  DynaCoMSettings settings_;
  Eigen::Matrix2d S_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  double mass_;
  double inf_ = std::numeric_limits<double>::infinity();

  // Lists of contacts
  // std::vector< std::shared_ptr<Contact6D> > known_contact6ds_;
  std::map<std::string, std::shared_ptr<Contact6D>> known_contact6ds_;
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

  Eigen::MatrixXd H_, C_, A_;
  Eigen::VectorXd g_, u_, l_;
  Eigen::Matrix<double, 6, 1> b_;
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
  std::vector<std::string>::iterator activeID_;
  std::map<std::string, std::shared_ptr<aig::Contact6D>>::iterator knownID_;

  void addSizes(const std::shared_ptr<Contact6D> &contact);
  void removeSizes(const std::shared_ptr<Contact6D> &contact);
  void resizeMatrices();
  void buildMatrices(const Eigen::Vector3d &groundCoMForce,
                     const Eigen::Vector3d &groundCoMTorque,
                     const Eigen::Vector3d &CoM);
  void solveQP();
  void distribute();

 public:
  DynaCoM();
  DynaCoM(const DynaCoMSettings settings);
  void initialize(const DynaCoMSettings settings);

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
                    const std::string &name, const bool active = true);
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
  const DynaCoMSettings &getSettings() { return settings_; }
  const pinocchio::Model &getModel() { return model_; }
  const pinocchio::Data &getData() { return data_; }

  const Eigen::MatrixXd &uni_A() { return unilaterality_A_; }
  const Eigen::VectorXd &uni_b() { return unilaterality_b_; }
  const Eigen::MatrixXd &fri_A() { return friction_A_; }
  const Eigen::VectorXd &fri_b() { return friction_b_; }
  const Eigen::VectorXd &reg_A() { return regularization_A_; }
  const Eigen::VectorXd &reg_b() { return regularization_b_; }
  const Eigen::Matrix<double, 6, -1> &NE_A() { return newton_euler_A_; }
  const Eigen::Matrix<double, 6, 1> &NE_b() { return newton_euler_b_; }
  const Eigen::VectorXd &allForces() { return F_; }
};

}  // namespace aig

#endif  // AIG_DYNACOM
