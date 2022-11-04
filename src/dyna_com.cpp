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
#include <eiquadprog/eiquadprog.hpp>
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
  Sz_.setZero();
  Sz_.diagonal().segment<4>(2) << 1, 1, 1, 1;

  // matrix_sizes
  uni_rows_ = 0;
  fri_rows_ = 0;
  cols_ = 0;
  newton_euler_b_.resize(6);
}

const Eigen::Matrix<double, 6, 6> DynaCoM::toWorldCoPWrench(
    pinocchio::SE3 pose) {
  /**
   * To compute any CoP, we need some surface. We compute the full
   * robot CoP considering always a horizontal surface. So, just
   * the vertical forces produce pressure on such surface.
   * This method generates the adjoint matrix needed to transform
   * local forces in some frame pose to a world wrench composed by
   * the vertical force and the torque. Lateral forces are discarted
   * because they do not contribute on this CoP.
   * Still, notice that the lateral forces have an effect that is
   * accounted in the non-linear effects.
   */

  soMs_ = pinocchio::SE3(pose.rotation(), Eigen::Vector3d::Zero());
  soXs_ = soMs_.toActionMatrixInverse().transpose();
  oMso_ = pinocchio::SE3(Eigen::Matrix3d::Identity(), pose.translation());
  oXso_ = oMso_.toActionMatrixInverse().transpose();

  return oXso_ * Sz_ * soXs_;
}

void DynaCoM::computeDynamics(const Eigen::VectorXd &posture,
                              const Eigen::VectorXd &velocity,
                              const Eigen::VectorXd &acceleration,
                              const Eigen::Matrix<double, 6, 1> &externalWrench,
                              bool flatHorizontalGround) {
  /**
   * @brief The external wrench is supposed to be expressed
   * in the frame of the Center of mass.
   *
   * @brief The option flatHorizontalGround assumes that supporting contacts
   * where previously defined.
   *
   * TODO: In the case when flatHorizontalGround = True, still, we could
   * remove the assumption of horizontal ground by including the lateral force
   * produced by the lateral components of the groundNormalReaction.
   *
   * For this we should know what is the direction normal to the ground. Then,
   * we could change the flag name by `bool flatGround`. The normal direction
   * can be obtained from the feet frames (both are the same in a flatGround)
   *
   */

  pinocchio::computeCentroidalMomentumTimeVariation(model_, data_, posture,
                                                    velocity, acceleration);

  acom_ = data_.dhg.linear() / mass_;
  dL_ = data_.dhg.angular();
  L_ = data_.hg.angular();

  groundCoMForce_ = data_.dhg.linear() - weight_ - externalWrench.head<3>();
  groundCoMTorque_ = dL_ - externalWrench.tail<3>();

  if (flatHorizontalGround)
    cop_ =
        data_.com[0].head<2>() + (S_ * groundCoMTorque_.head<2>() -
                                  groundCoMForce_.head<2>() * data_.com[0](2)) /
                                     (groundCoMForce_(2));

  else {
    distributeForce(groundCoMForce_, groundCoMTorque_, data_.com[0]);
    // @TODO: IT could happen that the expected groundCoMwrench is not feasible,
    // in such case, we should get the clossest possible and recompute the
    // centroidal motion accordingly. So, groundCoMForce_(2) in the following
    // should be updated.
    // The case when no contact is active should also be managed.

    CoPTorque_ = Eigen::Vector3d::Zero();
    for (std::string name : active_contact6ds_) {
      std::shared_ptr<Contact6D> &contact = known_contact6ds_[name];
      CoPTorque_ +=
          (toWorldCoPWrench(contact->getPose()) * contact->appliedForce())
              .segment<3>(3);
    }
    cop_ = S_ * CoPTorque_.head<2>() / groundCoMForce_(2);
  }
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
  contact->setPose(data_.oMf[contact->getFrameID()]);

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
    known_contact6ds_[name]->deactivate();
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

//void DynaCoM::solveQP() {
//  H_.resize(j_, j_);
//  g_.resize(j_);
//  C_.resize(uni_i_ + fri_i_, j_);
//  u_.resize(uni_i_ + fri_i_);
//  l_.resize(uni_i_ + fri_i_);
//  A_.resize(6, j_);

//  H_.setZero();
//  g_.setZero();
//  H_.diagonal() << (regularization_A_.cwiseAbs2()).segment(0, j_);
//  C_ << unilaterality_A_.block(0, 0, uni_i_, j_),
//      friction_A_.block(0, 0, fri_i_, j_);

//  u_.setZero();
//  l_.setConstant(-inf_);
//  A_ << newton_euler_A_.block(0, 0, 6, j_);
//  b_ << newton_euler_b_;
//  // Initialization of QP solver
//  // std::cout << "matrix H:\n " << H_ << std::endl;
//  // std::cout << "matrix A:\n " << A_ << std::endl;
//  // std::cout << "matrix b:\n " << b_ << std::endl;
//  // std::cout << "matrix C:\n " << C_ << std::endl;

//  // std::cout<<"In solveQP, matrices made, starting proxQP"<<std::endl;
//  proxsuite::proxqp::dense::isize dim = j_;
//  proxsuite::proxqp::dense::isize n_eq(6);
//  proxsuite::proxqp::dense::isize n_in(fri_i_ + uni_i_);
//  proxsuite::proxqp::dense::QP<double> qp(dim, n_eq, n_in);

//  qp.init(H_, g_, A_, b_, C_, l_,
//          u_);  //,std::nullopt,std::nullopt,std::nullopt
//  qp.solve();

//  F_.resize(j_);
//  F_ << qp.results.x;
//  // Check results
//  // std::cout << "solution:\n " << F_ << std::endl;
//}



void DynaCoM::solveQP() {
  /* eiquadprog formulation :
  min 0.5 * x G x + g0 x
  s.t.
  CE^T x + ce0 = 0
  CI^T x + ci0 >= 0
  */

  /* proxsuite formulation :
  min 0.5 * x H x + g^T x
  s.t.
  A x = b
  l <= C x <= u
  */

  int dim(static_cast<int>(j_)); // number of variables
  int n_eq(6); // number of equality constraints
  int n_ineq(static_cast<int>(fri_i_ + uni_i_)); // number of inequalities constraints

  F_.resize(dim);
  G_.resize(dim, dim);
  g0_.resize(dim);
  CE_.resize(dim, n_eq);
  ce0_.resize(n_eq);
  C_.resize(n_ineq, dim);
  CI_.resize(dim, n_ineq);
  ci0_.resize(n_ineq);

  G_.setZero();
  G_.diagonal() << (regularization_A_.cwiseAbs2()).segment(0, dim);
  g0_.setZero();

  CE_ << newton_euler_A_.block(0, 0, 6, dim).transpose();
  ce0_ << -newton_euler_b_;

  C_ << unilaterality_A_.block(0, 0, static_cast<int>(uni_i_), dim),
        friction_A_.block(0, 0, static_cast<int>(fri_i_), dim);
  CI_ = -C_.transpose();
  ci0_.setZero();

//  std::cout<<"G "<<std::endl<<G_<<std::endl;
//  std::cout<<"g0 "<<std::endl<<g0_<<std::endl;
//  std::cout<<"CE "<<std::endl<<CE_<<std::endl;
//  std::cout<<"ce "<<std::endl<<ce0_<<std::endl;
//  std::cout<<"CI "<<std::endl<<CI_<<std::endl;
//  std::cout<<"ci "<<std::endl<<ci0_<<std::endl;


  activeSetSize_ = 0;
  //const double precision =
  eiquadprog::solvers::solve_quadprog(G_, g0_, CE_, ce0_, CI_, ci0_, F_, activeSet_, activeSetSize_);
  //std::cout<<"DynaCom::SolveQP, finished with precision = "<<precision<<std::endl;
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
   * //@TODO: if the list of active contacts is empty, no force or torque
   * can be applied. Manage such case. We change the arguments? we throw
   * error? Check.
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
