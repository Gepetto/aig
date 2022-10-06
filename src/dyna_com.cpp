/**
 * @file
 * @copyright Copyright (c) 2022, LAAS-CNRS, Toward, PalRobotics
 * @brief
 */

#include "aig/dyna_com.hpp"

#include <algorithm>
#include <cctype>
#include <example-robot-data/path.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>

#include "aig/contact6d.hpp"

namespace aig {

DynaCoM::DynaCoM() {}
DynaCoM::DynaCoM(const DynaCoMSettings settings) { initialize(settings); }

void DynaCoM::initialize(const DynaCoMSettings settings) {
  // Copy the settings internally.
  settings_ = settings;

  // Check if the urdf and the srdf file exists or not.
  bool urdf_file_exists = false;
  {
    std::ifstream f(settings_.urdf.c_str());
    urdf_file_exists = f.good();
  }

  // Build the robot model.
  if (urdf_file_exists) {
    pinocchio::urdf::buildModel(settings_.urdf,
                                pinocchio::JointModelFreeFlyer(), model_);
  } else if (settings_.urdf != "") {
    pinocchio::urdf::buildModelFromXML(
        settings_.urdf, pinocchio::JointModelFreeFlyer(), model_);
  } else {
    throw std::runtime_error("DynaCoM::DynaCoM(): settings_.urdf is empty");
  }
  // Build pinocchio cache.
  data_ = pinocchio::Data(model_);

  mass_ = 0.0;
  for (size_t k = 0; k < model_.inertias.size(); ++k) {
    mass_ += model_.inertias[k].mass();
  }
  weight_ = mass_ * model_.gravity981;
  S_ << 0, -1, 1, 0;

  // matrix_sizes
  uni_rows_ = 0;
  fri_rows_ = 0;
  cols_ = 0;
  newton_euler_b_.resize(6);
}

void DynaCoM::computeDynamics(const Eigen::VectorXd &posture,
                              const Eigen::VectorXd &velocity,
                              const Eigen::VectorXd &acceleration,
                              const Eigen::Matrix<double, 6, 1> &externalWrench,
                              bool flatHorizontalGround) {
  // The external wrench is supposed to be expressed
  // in the frame of the Center of mass.
  pinocchio::computeCentroidalMomentumTimeVariation(model_, data_, posture,
                                                    velocity, acceleration);

  acom_ = data_.dhg.linear() / mass_;
  dL_ = data_.dhg.angular();
  L_ = data_.hg.angular();

  groundCoMForce_ = data_.dhg.linear() - weight_ - externalWrench.head<3>();
  groundCoMTorque_ = dL_ - externalWrench.tail<3>();

  if (flatHorizontalGround)
    nonCoPTorque_ = Eigen::Vector3d::Zero();
  else {
    // TODO get the force distribution and remove the non pressure terms from
    // the CoP computation. for now, we assume a flat and horizontal ground :
    nonCoPTorque_ = Eigen::Vector3d::Zero();
  }

  cop_ = data_.com[0].head<2>() +
         (S_ * groundCoMTorque_.head<2>() + nonCoPTorque_.head<2>() -
          groundCoMForce_.head<2>() * data_.com[0](2)) /
             (groundCoMForce_(2));
}

void DynaCoM::computeNL(const double &w, const Eigen::VectorXd &posture,
                        const Eigen::VectorXd &velocity,
                        const Eigen::VectorXd &acceleration,
                        const Eigen::Matrix<double, 6, 1> &externalWrench,
                        bool flatHorizontalGround) {
  computeDynamics(posture, velocity, acceleration, externalWrench,
                  flatHorizontalGround);
  computeNL(w);
}

void DynaCoM::computeNL(const double &w) {
  /**
   * In this function form, computeDynamics is suposed to have been called
   * before.
   */
  n_ = acom_.head<2>() / (w * w) - data_.com[0].head<2>() + cop_;
}

// Contact management
// //////////////////////////////////////////////////////////////////

void DynaCoM::addContact6d(const std::shared_ptr<Contact6D> &contact,
                           const std::string &name, const bool active) {
  contact->setFrameID(model_.getFrameId(contact->getSettings().frame_name));

  known_contact6ds_.insert(
      std::pair<std::string, std::shared_ptr<Contact6D>>(name, contact));

  addSizes(known_contact6ds_[name]);
  if (active) activateContact6d(name);
}

void DynaCoM::removeContact6d(const std::string &name) {
  knownID_ = known_contact6ds_.find(name);
  if (knownID_ != known_contact6ds_.end()) {
    removeSizes(known_contact6ds_[name]);
    deactivateContact6d(name);
    known_contact6ds_.erase(name);
  }
}

void DynaCoM::addSizes(const std::shared_ptr<Contact6D> &contact) {
  uni_rows_ += contact->uni_rows();
  fri_rows_ += contact->fri_rows();
  cols_ += contact->cols();

  resizeMatrices();
}

void DynaCoM::removeSizes(const std::shared_ptr<Contact6D> &contact) {
  uni_rows_ -= contact->uni_rows();
  fri_rows_ -= contact->fri_rows();
  cols_ -= contact->cols();

  resizeMatrices();
}

void DynaCoM::resizeMatrices() {
  unilaterality_A_.resize(uni_rows_, cols_);
  unilaterality_b_.resize(uni_rows_);
  friction_A_.resize(fri_rows_, cols_);
  friction_b_.resize(fri_rows_);
  regularization_A_.resize(cols_);
  regularization_b_.resize(cols_);
  newton_euler_A_.resize(6, cols_);
}

void DynaCoM::activateContact6d(const std::string &name) {
  activeID_ =
      std::find(active_contact6ds_.begin(), active_contact6ds_.end(), name);
  knownID_ = known_contact6ds_.find(name);

  if (activeID_ == active_contact6ds_.end()) {
    if (knownID_ != known_contact6ds_.end()) {
      active_contact6ds_.push_back(name);
      std::cout << "activated contact " << name << std::endl;
      return;
    } else {
      std::cout << "no contact called " << name << " was defined" << std::endl;
      return;
    }
  }
  std::cout << name << " was already active" << std::endl;
}

void DynaCoM::deactivateContact6d(const std::string &name) {
  activeID_ =
      std::find(active_contact6ds_.begin(), active_contact6ds_.end(), name);
  if (activeID_ != active_contact6ds_.end()) {
    active_contact6ds_.erase(activeID_);
    std::cout << "deactivated contact " << name << std::endl;
    return;
  }
  std::cout << name << " was not active" << std::endl;
}

void DynaCoM::buildMatrices(const Eigen::Vector3d &groundCoMForce,
                            const Eigen::Vector3d &groundCoMTorque,
                            const Eigen::Vector3d &CoM) {
  size_t uni_r, fri_r, cols;

  uni_i_ = 0;
  fri_i_ = 0;
  j_ = 0;
  for (std::string name : active_contact6ds_) {
    std::shared_ptr<Contact6D> &contact = known_contact6ds_[name];

    uni_r = contact->uni_rows();
    fri_r = contact->fri_rows();
    cols = contact->cols();

    unilaterality_A_.block(uni_i_, j_, uni_r, cols) << contact->uni_A();
    unilaterality_A_.block(0, j_, uni_i_, cols).setZero();
    unilaterality_A_.block(uni_i_, 0, uni_r, j_).setZero();
    unilaterality_b_.segment(uni_i_, uni_r) << contact->uni_b();

    friction_A_.block(fri_i_, j_, fri_r, cols) << contact->fri_A();
    friction_A_.block(0, j_, fri_i_, cols).setZero();
    friction_A_.block(fri_i_, 0, fri_r, j_).setZero();
    friction_b_.segment(fri_i_, fri_r) << contact->fri_b();

    regularization_A_.segment(j_, cols) << contact->reg_A();
    regularization_b_.segment(j_, cols) << contact->reg_b();

    contact->updateNewtonEuler(CoM, pinocchio::updateFramePlacement(
                                        model_, data_, contact->getFrameID()));
    newton_euler_A_.block(0, j_, 6, cols) << contact->NE_A();

    uni_i_ += uni_r;
    fri_i_ += fri_r;
    j_ += cols;
  }
  newton_euler_b_ << groundCoMForce, groundCoMTorque;
}

void DynaCoM::solveQP() {
  H_.resize(j_, j_);
  g_.resize(j_);
  C_.resize(uni_i_ + fri_i_, j_);
  u_.resize(uni_i_ + fri_i_);
  l_.resize(uni_i_ + fri_i_);
  A_.resize(6, j_);

  H_.setZero();
  g_.setZero();
  H_.diagonal() << (regularization_A_.cwiseAbs2()).segment(0, j_);
  C_ << unilaterality_A_.block(0, 0, uni_i_, j_),
      friction_A_.block(0, 0, fri_i_, j_);

  u_.setZero();
  l_.setConstant(-inf_);
  A_ << newton_euler_A_.block(0, 0, 6, j_);
  b_ << newton_euler_b_;
  // Initialization of QP solver
  // std::cout << "matrix H:\n " << H_ << std::endl;
  // std::cout << "matrix A:\n " << A_ << std::endl;
  // std::cout << "matrix b:\n " << b_ << std::endl;
  // std::cout << "matrix C:\n " << C_ << std::endl;

  // std::cout<<"In solveQP, matrices made, starting proxQP"<<std::endl;
  proxsuite::proxqp::dense::isize dim = j_;
  proxsuite::proxqp::dense::isize n_eq(6);
  proxsuite::proxqp::dense::isize n_in(fri_i_ + uni_i_);
  proxsuite::proxqp::dense::QP<double> qp(dim, n_eq, n_in);

  qp.init(H_, g_, A_, b_, C_, u_, l_);  // std::nullopt
  qp.solve();

  F_.resize(j_);
  F_ << qp.results.x;
  // std::cout << "results.x: " << qp.results.x << std::endl;
  // std::cout << "results.y: " << qp.results.y << std::endl;
  // std::cout << "results.z: " << qp.results.z << std::endl;

  ////////////////////////////////////////////////////////////////
  // Eigen::Matrix2d H = Eigen::Matrix2d::Identity();
  // Eigen::Vector2d g = Eigen::Vector2d::Zero();

  // Eigen::Matrix2d C = -Eigen::Matrix2d::Identity();
  // Eigen::Vector2d u = Eigen::Vector2d::Zero();
  // Eigen::Vector2d l = Eigen::Vector2d::Constant(-10);

  // Eigen::Matrix2d A = Eigen::Matrix2d::Identity();
  // Eigen::Vector2d b;
  // b << 0, 1;

  // std::cout << "matrix H:\n " << H << std::endl;
  // std::cout << "matrix A:\n " << A << std::endl;
  // std::cout << "matrix b:\n " << b << std::endl;
  // std::cout << "matrix C:\n " << C << std::endl;
  // std::cout << "matrix u:\n " << u << std::endl;

  // proxsuite::proxqp::dense::isize dim = 2;
  // proxsuite::proxqp::dense::isize n_eq(2);
  // proxsuite::proxqp::dense::isize n_in(2);
  // proxsuite::proxqp::dense::QP<double> qp(dim, n_eq, n_in);

  // qp.init(H, g, A, b, C, u, l); //std::nullopt
  // qp.solve();

  // std::cout << "results.x: " << qp.results.x << std::endl;
  // std::cout << "results.y: " << qp.results.y << std::endl;
  // std::cout << "results.z: " << qp.results.z << std::endl;
  // std::cout << "nada " << std::endl;
}

void DynaCoM::distribute() {
  Eigen::Index n, i = 0;
  for (std::string name : active_contact6ds_) {
    std::shared_ptr<Contact6D> &contact = known_contact6ds_[name];

    n = contact->cols();
    contact->applyForce(F_.segment(i, n));
    i += n;
  }
}

void DynaCoM::distributeForce(const Eigen::Vector3d &groundCoMForce,
                              const Eigen::Vector3d &groundCoMTorque,
                              const Eigen::Vector3d &CoM) {
  /**
   *
   *  Make sure that the data of the dynaCoM
   * class has been updated to the correct robot
   * posture before executing this distribution.
   *
   * */
  buildMatrices(groundCoMForce, groundCoMTorque, CoM);
  // std::cout<<"it is at least building the matrices"<<std::endl;
  solveQP();
  // std::cout<<"it is actually solving the QP"<<std::endl;
  distribute();
  // std::cout<<"it even distribute the forces"<<std::endl;
  std::cout << "Distributed" << std::endl;
}

}  // namespace aig