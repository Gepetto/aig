#ifndef __PREVIEW_IK_CONFIGURATION___
#define __PREVIEW_IK_CONFIGURATION___
#include <iostream>
#include "example-robot-data/path.hpp"

namespace conf{
    const std::string path_to_robots (EXAMPLE_ROBOT_DATA_MODEL_DIR);
    const std::string urdf_path = path_to_robots + "/talos_data/robots/talos_reduced_corrected.urdf";
    const std::string srdf_path = path_to_robots + "/talos_data/srdf/talos.srdf";
}

#endif // __PREVIEW_IK_CONFIGURATION