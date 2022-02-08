/**
 * @file
 * @copyright Copyright (c) 2022, LAAS-CNRS, Toward, PalRobotics
 * @brief
 */

#include "preview_ik/leg_ig.hpp"

namespace preview_ik {

LegIG::LegIG() {}

LegIG::LegIG(const LegIGSettings &settings) { initialize(settings); }

void LegIG::initialize(const LegIGSettings &settings) { settings_ = settings; }

LegJoints LegIG::solve(const pinocchio::SE3 &base,
                       const pinocchio::SE3 &endEffector) {
  Eigen::Vector3d hip =
      base.translation() + base.rotation() * settings_.hip_from_waist;
  Eigen::Vector3d ankle = endEffector.translation() +
                          endEffector.rotation() * settings_.ankle_from_foot;
  Eigen::Vector3d hipFromAnkle = hip - ankle;
  double distance_hip_ankle = hipFromAnkle.norm();

  double q2, q3, q4, q5, q6, q7;
  double cos_q5 = (pow(distance_hip_ankle, 2) - pow(settings_.femur_length, 2) -
                   pow(settings_.tibia_length, 2)) /
                  (2.0 * settings_.femur_length * settings_.tibia_length);
  if (cos_q5 >= 1)
    q5 = 0;
  else if (cos_q5 <= -1)
    q5 = M_PI;
  else
    q5 = acos(cos_q5);

  q7 = atan2(hipFromAnkle(1), hipFromAnkle(2));
  if (q7 > M_PI_2)
    q7 -= M_PI;
  else if (q7 < -M_PI_2)
    q7 += M_PI;

  int zDirectionLeg;
  hipFromAnkle(2) > 0 ? zDirectionLeg = 1 : zDirectionLeg = -1;

  q6 = -atan2(hipFromAnkle(0), zDirectionLeg * hipFromAnkle.tail<2>().norm()) -
       asin((settings_.femur_length / distance_hip_ankle) * sin(M_PI - q5));

  Eigen::Matrix3d Rint, Rext, R;
  Rext = base.rotation().transpose() * endEffector.rotation();
  Rint = Eigen::AngleAxisd(-q7, Eigen::Vector3d(1, 0, 0)) *
         Eigen::AngleAxisd(-q5 - q6, Eigen::Vector3d(0, 1, 0));
  R = Rext * Rint;
  q2 = atan2(-R(0, 1), R(1, 1));
  q3 = atan2(R(2, 1), -R(0, 1) * sin(q2) + R(1, 1) * cos(q2));
  q4 = atan2(-R(2, 0), R(2, 2));

  LegJoints leg;
  leg << q2, q3, q4, q5, q6, q7;
  return leg;
}

}  // namespace preview_ik