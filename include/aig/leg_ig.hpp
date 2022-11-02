/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Class to perform inverse geometry on a robot leg.
 */

#ifndef AIG_LEG_IG
#define AIG_LEG_IG

// clang-format off
#include <Eigen/Dense>

#include "pinocchio/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
// clang-format on

namespace aig {

typedef Eigen::Matrix<double, 6, 1> LegJoints;

/**
 * @brief
 */
struct LegIGSettings {
 public:
  Eigen::Vector3d hip_from_waist;
  Eigen::Vector3d knee_from_hip;
  Eigen::Vector3d ankle_from_knee;
  Eigen::Vector3d ankle_from_foot;

  LegIGSettings()
    : hip_from_waist(Eigen::Vector3d::Zero())
    , knee_from_hip(Eigen::Vector3d::Zero())
    , ankle_from_knee(Eigen::Vector3d::Zero())
    , ankle_from_foot(Eigen::Vector3d::Zero())
  {
  }


  LegIGSettings(const Eigen::Vector3d &_hip_from_waist, const Eigen::Vector3d &_knee_from_hip,
                const Eigen::Vector3d &_ankle_from_knee, const Eigen::Vector3d &_ankle_from_foot)
    : hip_from_waist(_hip_from_waist)
    , knee_from_hip(_knee_from_hip)
    , ankle_from_knee(_ankle_from_knee)
    , ankle_from_foot(_ankle_from_foot)
  {
  }

  friend std::ostream &operator<<(std::ostream &out, const LegIGSettings &obj)
  {
    out << "LegIGSettings:\n"
        << "    hip_from_waist: " << obj.hip_from_waist.transpose() << "\n"
        << "    knee_from_hip: " << obj.knee_from_hip.transpose() << "\n"
        << "    ankle_from_knee: " << obj.ankle_from_knee.transpose() << "\n"
        << "    ankle_from_foot: " << obj.ankle_from_foot.transpose()
        << std::endl;
    return out;
  }

  friend bool operator==(const LegIGSettings &lhs, const LegIGSettings &rhs) {
    bool test = true;
    test &= lhs.hip_from_waist == rhs.hip_from_waist;
    test &= lhs.knee_from_hip == rhs.knee_from_hip;
    test &= lhs.ankle_from_knee == rhs.ankle_from_knee;
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

  // internals
  Eigen::Vector3d hip_, ankle_, hip_from_ankle_;
  double epsilon_, c5_;
  double q2_, q3_, q4_, q5_, q6_, q7_;
  double sign_hip_from_ankle_z;
  double a_, b_, c_;
  Eigen::Matrix3d Rint_, Rext_, R_;
  LegJoints output_;

 public:
  LegIG();
  LegIG(const LegIGSettings &settings);
  void reset_internals();
  const LegIGSettings &get_settings() { return settings_; }
  void initialize(const LegIGSettings &settings);
  LegJoints solve(const pinocchio::SE3 &base,
                  const pinocchio::SE3 &endEffector);
};
}  // namespace aig

#endif  // AIG_LEG_IG
