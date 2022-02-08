/**
 * @file
 * @copyright Copyright (c) 2022, LAAS-CNRS, Toward, PalRobotics
 * @brief
 */

#ifndef NDEBUG  // debug mode
#include <iostream>
#endif
#include "preview_ik/arm_ig.hpp"

namespace preview_ik {

ArmIG::ArmIG() {
#ifndef NDEBUG  // debug mode
  std::cout << "DEBUG: ArmIG::ArmIG(): Not implemented" << std::endl;
#endif
}

Eigen::VectorXd ArmIG::solve(const pinocchio::SE3& /*base*/,
                             const pinocchio::SE3& /*endEffector*/) {
#ifndef NDEBUG  // debug mode
  std::cout << "DEBUG: ArmIG::solve(): Not implemented" << std::endl;
#endif
  Eigen::VectorXd q;
  return q;
}

}  // namespace preview_ik