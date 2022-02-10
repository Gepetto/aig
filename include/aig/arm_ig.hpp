/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Class to perform inverse geometry on a robot arm.
 */

#ifndef AIG_ARM_IG
#define AIG_ARM_IG

#include <Eigen/Dense>

#include "pinocchio/spatial/se3.hpp"

namespace aig {
/**
 * @brief @todo Describe ArmIG
 */
class ArmIG {
 public:
  ArmIG();
  Eigen::VectorXd solve(const pinocchio::SE3 &base,
                        const pinocchio::SE3 &endEffector);
};
}  // namespace aig

#endif  // AIG_ARM_IG