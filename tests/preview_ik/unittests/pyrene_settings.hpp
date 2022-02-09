/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Description of the robot Pyrene's (Talos model of PalRobotics in
 *        LAAS-CNRS) parameters in order to perform the inverse geometry (IG).
 */

#ifndef PREVIEW_IK_PYRENE_SETTINGS
#define PREVIEW_IK_PYRENE_SETTINGS

#include <string>

#include "preview_ik/leg_ig.hpp"
#include "example-robot-data/path.hpp"

namespace preview_ik {
namespace unittests {
const std::string path_to_robots = EXAMPLE_ROBOT_DATA_MODEL_DIR;
const std::string urdf_path =
    path_to_robots + "/talos_data/robots/talos_reduced_corrected.urdf";
const std::string srdf_path = path_to_robots + "/talos_data/srdf/talos.srdf";

const std::string left_hip_joint_name = "leg_left_1_joint";
const std::string right_hip_joint_name = "leg_right_1_joint";
const std::string left_knee_joint_name = "leg_left_4_joint";
const std::string right_knee_joint_name = "leg_right_4_joint";
const std::string left_ankle_joint_name = "leg_left_5_joint";
const std::string right_ankle_joint_name = "leg_right_5_joint";
const std::string left_foot_frame_name = "leg_left_sole_fix_joint";
const std::string right_foot_frame_name = "leg_right_sole_fix_joint";

// hip_from_waist
// knee_from_hip
// ankle_from_knee
// ankle_from_foot
const LegIGSettings llegs = {
    (Eigen::Vector3d() << -0.02, 0.085, -0.27105).finished(),
    (Eigen::Vector3d() << 0.0, 0.0, -0.38).finished(),
    (Eigen::Vector3d() << 0.0, 0.0, -0.325).finished(),
    (Eigen::Vector3d() << 0.0, 0.0, 0.107).finished()
};

const LegIGSettings rlegs = {
    (Eigen::Vector3d() << -0.02, -0.085, -0.27105).finished(),
    (Eigen::Vector3d() << 0.0, 0.0, -0.38).finished(),
    (Eigen::Vector3d() << 0.0, 0.0, -0.325).finished(),
    (Eigen::Vector3d() << 0.0, 0.0, 0.107).finished()
};

}  // namespace unittests
}  // namespace preview_ik

#endif  // PREVIEW_IK_PYRENE_SETTINGS