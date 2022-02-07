/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Description of the robot Pyrene's (Talos model of PalRobotics in
 *        LAAS-CNRS) parameters in order to perform the inverse geometry (IG).
 */

#ifndef PREVIEW_IK_CONFIGURATION
#define PREVIEW_IK_CONFIGURATION

#include <string>

#include "example-robot-data/path.hpp"

namespace preview_ik {
namespace unittests {
const std::string path_to_robots(EXAMPLE_ROBOT_DATA_MODEL_DIR);
const std::string urdf_path = path_to_robots + "/talos_data/robots/talos_reduced_corrected.urdf";
const std::string srdf_path = path_to_robots + "/talos_data/srdf/talos.srdf";

const std::string left_hip_jointName = "leg_left_1_joint";
const std::string right_hip_jointName = "leg_right_1_joint";
const std::string left_knee_jointName = "leg_left_4_joint";
const std::string right_knee_jointName = "leg_right_4_joint";
const std::string left_ankle_jointName = "leg_left_5_joint";
const std::string right_ankle_jointName = "leg_right_5_joint";
const std::string left_foot_frameName = "leg_left_sole_fix_joint";
const std::string right_foot_frameName = "leg_right_sole_fix_joint";
}  // namespace unittests
}  // namespace preview_ik

#endif  // PREVIEW_IK_CONFIGURATION