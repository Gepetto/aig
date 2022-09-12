/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Description of the robot Pyrene's (Talos model of PalRobotics in
 *        LAAS-CNRS) parameters in order to perform the inverse geometry (IG).
 */

#ifndef AIG_PYRENE_SETTINGS
#define AIG_PYRENE_SETTINGS

#include <string>

#include "aig/biped_ig.hpp"
#include "aig/leg_ig.hpp"
#include "example-robot-data/path.hpp"

namespace aig {
namespace unittests {
const std::string path_to_robots = EXAMPLE_ROBOT_DATA_MODEL_DIR;
const std::string urdf =
    path_to_robots + "/talos_data/robots/talos_reduced_corrected.urdf";
const std::string srdf = path_to_robots + "/talos_data/srdf/talos.srdf";

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
    (Eigen::Vector3d() << -0.02, 0.085, -0.27105)
        .finished(),                                    /* hip_from_waist */
    (Eigen::Vector3d() << 0.0, 0.0, -0.38).finished(),  /* knee_from_hip */
    (Eigen::Vector3d() << 0.0, 0.0, -0.325).finished(), /* ankle_from_knee */
    (Eigen::Vector3d() << 0.0, 0.0, 0.107).finished()   /* ankle_from_foot */
};

const LegIGSettings rlegs = {
    (Eigen::Vector3d() << -0.02, -0.085, -0.27105).finished(),
    (Eigen::Vector3d() << 0.0, 0.0, -0.38).finished(),
    (Eigen::Vector3d() << 0.0, 0.0, -0.325).finished(),
    (Eigen::Vector3d() << 0.0, 0.0, 0.107).finished()};

aig::BipedIGSettings bipeds = {
    left_hip_joint_name,    /* left_hip_joint_name */
    left_knee_joint_name,   /* left_knee_joint_name */
    left_ankle_joint_name,  /* left_ankle_joint_name */
    left_foot_frame_name,   /* left_foot_frame_name */
    right_hip_joint_name,   /* right_hip_joint_name */
    right_knee_joint_name,  /* right_knee_joint_name */
    right_ankle_joint_name, /* right_ankle_joint_name */
    right_foot_frame_name,  /* right_foot_frame_name */
    urdf,                   /* urdf paths */
    srdf                    /* srdf paths */
};

}  // namespace unittests
}  // namespace aig

#endif  // AIG_PYRENE_SETTINGS
