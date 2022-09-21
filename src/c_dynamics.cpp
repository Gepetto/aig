/**
 * @file
 * @copyright Copyright (c) 2022, LAAS-CNRS, Toward, PalRobotics
 * @brief
 */

#include "aig/c_dynamics.hpp"

#include <algorithm>
#include <cctype>

#include "example-robot-data/path.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace dyno {

Contact6D::Contact6D() {}
Contact6D::Contact6D(const Contact6DSettings &settings) {
  initialize(settings);
}
void Contact6D::initialize(const Contact6DSettings &settings) {
  settings_ = settings;
  double hl = settings_.half_length;
  double hw = settings_.half_width;
  double mu = settings_.mu;
  double gu = settings_.gu;

  // Make matrices
  regularization_A_ << settings_.weights;
  regularization_b_ << Eigen::Matrix<double, 6, 1>::Zero();

  unilaterality_A_ << Eigen::Matrix<double, 5, 6>::Zero();
  unilaterality_A_.block<5, 1>(0, 2) << -1, -hl, -hw, -hl, -hw;
  unilaterality_A_.block<2, 2>(1, 3) << 0, -1, 1, 0;
  unilaterality_A_.block<2, 2>(3, 3) << 0, -1, 1, 0;
  unilaterality_b_ << Eigen::Matrix<double, 6, 1>::Zero();

  friction_A_ << Eigen::Matrix<double, 6, 6>::Zero();
  friction_A_.block<6, 1>(0, 2) << -mu, -mu, -mu, -mu, -gu, -gu;
  friction_A_.block<2, 2>(0, 0) << Eigen::Matrix2d::Identity();
  friction_A_.block<2, 2>(2, 0) << -Eigen::Matrix2d::Identity();
  friction_A_.block<2, 1>(4, 5) << 1, -1;
  friction_b_ << Eigen::Matrix<double, 6, 1>::Zero();

  newton_euler_A_ << Eigen::Matrix<double, 6, 6>::Zero();
  newton_euler_A_.block<3, 3>(0, 0) << Eigen::Matrix3d::Identity();
  newton_euler_A_.block<3, 3>(3, 3) << Eigen::Matrix3d::Identity();

  // Note: newton_euler must be updated before using it

  contactForce_ = Eigen::Matrix<double, 6, 1>::Zero();
}

void Contact6D::active(const bool &active) {
  settings_.active = active;
  if (!active) contactForce_.setZero();
}

void Contact6D::setForceWeights(const Eigen::Vector3d &force_weights) {
  settings_.weights.head<3>() = force_weights;
  regularization_A_.head<3>() = force_weights;
}

void Contact6D::setTorqueWeights(const Eigen::Vector3d &torque_weights) {
  settings_.weights.tail<3>() = torque_weights;
  regularization_A_.tail<3>() = torque_weights;
}

void Contact6D::setSurfaceHalfWidth(const double &half_width) {
  settings_.half_width = half_width;
  unilaterality_A_(2, 2) = -half_width;
  unilaterality_A_(4, 2) = -half_width;
}

void Contact6D::setSurfaceHalfLength(const double &half_length) {
  settings_.half_length = half_length;
  unilaterality_A_(1, 2) = -half_length;
  unilaterality_A_(3, 2) = -half_length;
}

void Contact6D::setMu(const double &mu) {
  settings_.mu = mu;
  friction_A_.block<4, 1>(0, 2) << -mu, -mu, -mu, -mu;
}

void Contact6D::setGu(const double &gu) {
  settings_.gu = gu;
  friction_A_.block<2, 1>(4, 2) << -gu, -gu;
}

void Contact6D::updateNewtonEuler(const Eigen::Vector3d &CoM,
                                  const pinocchio::SE3 &oMf) {
  newton_euler_A_.block<3, 3>(3, 0) << pinocchio::skew(oMf.translation() - CoM);
  oAf_ = oMf.toActionMatrixInverse().transpose();
}

Dyno::Dyno() {}
Dyno::Dyno(const DynoSettings settings) { initialize(settings); }

void Dyno::initialize(const DynoSettings settings) {
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
    throw std::runtime_error("Dyno::Dyno(): settings_.urdf is empty");
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

void Dyno::computeDynamics(const Eigen::VectorXd &posture,
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

void Dyno::computeNL(const double &w, const Eigen::VectorXd &posture,
                     const Eigen::VectorXd &velocity,
                     const Eigen::VectorXd &acceleration,
                     const Eigen::Matrix<double, 6, 1> &externalWrench,
                     bool flatHorizontalGround) {
  computeDynamics(posture, velocity, acceleration, externalWrench,
                  flatHorizontalGround);
  computeNL(w);
}

void Dyno::computeNL(const double &w) {
  /**
   * In this function form, computeDynamics is suposed to have been called
   * before.
   */
  n_ = acom_.head<2>() / (w * w) - data_.com[0].head<2>() + cop_;
}

// Contact management
// //////////////////////////////////////////////////////////////////

void Dyno::addContact6d(const std::shared_ptr<Contact6D> &contact,
                        const std::string &name) {
  contact->setFrameID(model_.getFrameId(contact->getSettings().frame_name));

  known_contact6ds_.insert(
      std::pair<std::string, std::shared_ptr<Contact6D>>(name, contact));

  addSizes(known_contact6ds_[name]);
  if (known_contact6ds_[name]->getSettings().active) activateContact6d(name);
}

void Dyno::removeContact6d(const std::string &name) {
  removeSizes(known_contact6ds_[name]);
  if (known_contact6ds_[name]->getSettings().active) deactivateContact6d(name);

  known_contact6ds_.erase(name);
}

void Dyno::addSizes(const std::shared_ptr<Contact6D> &contact) {
  uni_rows_ += contact->uni_rows();
  fri_rows_ += contact->fri_rows();
  cols_ += contact->cols();

  resizeMatrices();
}

void Dyno::removeSizes(const std::shared_ptr<Contact6D> &contact) {
  uni_rows_ -= contact->uni_rows();
  fri_rows_ -= contact->fri_rows();
  cols_ -= contact->cols();

  resizeMatrices();
}

void Dyno::resizeMatrices() {
  unilaterality_A_.resize(uni_rows_, cols_);
  unilaterality_b_.resize(uni_rows_);
  friction_A_.resize(fri_rows_, cols_);
  friction_b_.resize(fri_rows_);
  regularization_A_.resize(cols_);
  regularization_b_.resize(cols_);
  newton_euler_A_.resize(6, cols_);
}

void Dyno::activateContact6d(const std::string &name) {
  known_contact6ds_[name]->active(true);

  active_contact6ds_.push_back(name);
}

void Dyno::deactivateContact6d(const std::string &name) {
  known_contact6ds_[name]->active(false);

  for (auto it = active_contact6ds_.begin(); it != active_contact6ds_.end();
       it++) {
    if (*it == name) active_contact6ds_.erase(it);
  }
}

void Dyno::buildMatrices(const Eigen::Vector3d &groundCoMForce,
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

    contact->updateNewtonEuler(CoM, data_.oMf[contact->getFrameID()]);

    unilaterality_A_.block(uni_i_, j_, uni_r, cols) << contact->uni_A();
    friction_A_.block(fri_i_, j_, fri_r, cols) << contact->fri_A();
    regularization_A_.segment(j_, cols) << contact->reg_A();
    newton_euler_A_.block(0, j_, 6, cols)
        << contact->NE_A() * contact->toWorldForces();

    unilaterality_b_.segment(uni_i_, uni_r) << contact->uni_b();
    friction_b_.segment(fri_i_, fri_r) << contact->fri_b();
    regularization_b_.segment(j_, cols) << contact->reg_b();

    uni_i_ += uni_r;
    fri_i_ += fri_r;
    j_ += cols;
  }
  newton_euler_b_ << groundCoMForce, groundCoMTorque;
}

void Dyno::solveQP() {
  G_.resize(j_, j_);
  CI_.resize(uni_i_ + fri_i_, j_);
  CE_.resize(6, j_);

  G_ << (regularization_A_.cwiseSqrt()).diagonal().block(0, 0, j_, j_);
  CI_ << -unilaterality_A_.transpose().block(0, 0, uni_i_, j_),
      -friction_A_.transpose().block(0, 0, fri_i_, j_);
  CE_ << newton_euler_A_.transpose().block(0, 0, 6, j_);

  F_.resize(j_);
  F_.setZero();  // replace it by the QP solver
}

void Dyno::distribute() {
  int i = 0;
  long n;
  for (std::string name : active_contact6ds_) {
    std::shared_ptr<Contact6D> &contact = known_contact6ds_[name];

    n = contact->cols();
    contact->applyForce(F_.segment(i, n));
  }
}

void Dyno::distributeForce(const Eigen::Vector3d &groundCoMForce,
                           const Eigen::Vector3d &groundCoMTorque,
                           const Eigen::Vector3d &CoM) {
  buildMatrices(groundCoMForce, groundCoMTorque, CoM);
  solveQP();
  distribute();
}

}  // namespace dyno
