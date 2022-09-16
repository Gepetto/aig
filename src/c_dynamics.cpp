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

  Dyno::Dyno(){}
  Dyno::Dyno(const DynoSettings settings){initialize(settings);}

  void Dyno::initialize(const DynoSettings settings){
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
}

