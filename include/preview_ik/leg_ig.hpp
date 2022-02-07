/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Class to perform inverse geometry on a robot leg.
 */

#ifndef PREVIEW_IK_LEG_IG
#define PREVIEW_IK_LEG_IG

#include <Eigen/Dense>

#include "pinocchio/spatial/se3.hpp"

namespace preview_ik {

typedef Eigen::Matrix<double, 6, 1> LegJoints;

/**
 * @brief
 */
struct LegSettings {
 public:
  enum Side { LEFT, RIGHT };

 public:
  Side side = LEFT;
  double femur_length = 0.0;
  double tibia_length = 0.0;
  Eigen::Vector3d hip_from_waist = Eigen::Vector3d::Zero();
  Eigen::Vector3d ankle_from_foot = Eigen::Vector3d::Zero();
};

/**
 * @brief @todo
 */
class LegIG {
 private:
  LegSettings settings_;

 public:
  LegIG();
  LegIG(const LegSettings& settings);
  void initialize(const LegSettings& settings);
  LegJoints solve(const pinocchio::SE3& base, const pinocchio::SE3& endEffector);
};
}  // namespace preview_ik

#endif  // PREVIEW_IK_LEG_IG