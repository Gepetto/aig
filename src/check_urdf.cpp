#include "pinocchio/parsers/urdf.hpp"
#include "example-robot-data/path.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"


#include<iostream>


std::string path_to_all_urdf (EXAMPLE_ROBOT_DATA_MODEL_DIR);

namespace pin = pinocchio;


int main(){

    const std::string urdf_path = path_to_all_urdf + "/talos_data/robots/talos_reduced_corrected.urdf";

    pin::Model model;
    //pin::urdf::buildModel(urdf_path, model, false);
    //model.loadFromString()
    //pin::urdf::buildModel
    
    std::cout << "\n\n" << urdf_path << "\n\n" << std::endl;

}

