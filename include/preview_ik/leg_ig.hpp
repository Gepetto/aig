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
struct LegIGSettings {
 public:
  enum Side { LEFT, RIGHT };

 public:
  Side side = LEFT;
  double femur_length = 0.0;
  double tibia_length = 0.0;
  Eigen::Vector3d hip_from_waist = Eigen::Vector3d::Zero();
  Eigen::Vector3d ankle_from_foot = Eigen::Vector3d::Zero();

  friend std::ostream &operator<<(std::ostream &out, const LegIGSettings &obj) {
    out << "LegIGSettings:\n";
    out << "    side: ";
    obj.side == LEFT ? out << "LEFT" : out << "RIGHT";
    out << "\n"
        << "    femur_length: " << obj.femur_length << "\n"
        << "    tibia_length: " << obj.tibia_length << "\n"
        << "    hip_from_waist: " << obj.hip_from_waist.transpose() << "\n"
        << "    ankle_from_foot: " << obj.ankle_from_foot.transpose()
        << std::endl;
    return out;
  }

  friend bool operator==(const LegIGSettings &lhs, const LegIGSettings &rhs) {
    bool test = true;
    test &= lhs.side == rhs.side;
    test &= lhs.femur_length == rhs.femur_length;
    test &= lhs.tibia_length == rhs.tibia_length;
    test &= lhs.hip_from_waist == rhs.hip_from_waist;
    test &= lhs.ankle_from_foot == rhs.ankle_from_foot;
    return test;
  }
};

/**
 * @brief @todo
 */
class LegIG {
 private:
  LegIGSettings settings_;

 public:
  LegIG();
  LegIG(const LegIGSettings &settings);
  const LegIGSettings &get_settings() { return settings_; }
  void initialize(const LegIGSettings &settings);
  LegJoints solve(const pinocchio::SE3 &base,
                  const pinocchio::SE3 &endEffector);
};
}  // namespace preview_ik

#endif  // PREVIEW_IK_LEG_IG