#ifndef __PREVIEW_IK_CONFIGURATION___
#define __PREVIEW_IK_CONFIGURATION___
#include <iostream>

// clang-format off
#include "pinocchio/fwd.hpp"
// clang-format on

namespace conf {
const std::string path_to_robots(EXAMPLE_ROBOT_DATA_MODEL_DIR);
const std::string urdf_path =
    path_to_robots + "/talos_data/robots/talos_reduced_corrected.urdf";
const std::string srdf_path = path_to_robots + "/talos_data/srdf/talos.srdf";

const std::string leftHipJointName = "leg_left_1_joint";
const std::string rightHipJointName = "leg_right_1_joint";
const std::string leftKneeJointName = "leg_left_4_joint";
const std::string rightKneeJointName = "leg_right_4_joint";
const std::string leftAnkleJointName = "leg_left_5_joint";
const std::string rightAnkleJointName = "leg_right_5_joint";
const std::string leftFootFrameName = "leg_left_sole_fix_joint";
const std::string rightFootFrameName = "leg_right_sole_fix_joint";

} // namespace conf

#endif // __PREVIEW_IK_CONFIGURATION
