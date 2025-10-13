/**
 * @file
 * @copyright Copyright (c) 2022, LAAS-CNRS, Toward, PalRobotics
 * @brief
 */

#ifndef NDEBUG // debug mode
#include <iostream>
#endif
#include "aig/arm_ig.hpp"

namespace aig {

ArmIG::ArmIG() {
#ifndef NDEBUG // debug mode
  std::cout << "DEBUG: ArmIG::ArmIG(): Not implemented" << std::endl;
#endif
}

Eigen::VectorXd ArmIG::solve(const pinocchio::SE3 & /*base*/,
                             const pinocchio::SE3 & /*endEffector*/) {
#ifndef NDEBUG // debug mode
  std::cout << "DEBUG: ArmIG::solve(): Not implemented" << std::endl;
#endif
  Eigen::VectorXd q;
  return q;
}

} // namespace aig
