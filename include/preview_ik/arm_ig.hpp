/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Class to perform inverse geometry on a robot arm.
 */

#ifndef PREVIEW_IK_ARM_IG
#define PREVIEW_IK_ARM_IG

#include <Eigen/Dense>

#include "pinocchio/spatial/se3.hpp"

namespace preview_ik {
/**
 * @brief @todo Describe ArmIG
 */
class ArmIG {
 public:
  ArmIG();
  Eigen::VectorXd solve(const pinocchio::SE3 &base, const pinocchio::SE3 &endEffector);
};
}  // namespace preview_ik

#endif  // PREVIEW_IK_ARM_IG