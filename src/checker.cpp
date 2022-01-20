#include<iostream>
#include<Eigen/Dense>
#include"../include/preview_IK/postures.hpp"

#include "example-robot-data/path.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include<iostream>


std::string path_to_all_urdf (EXAMPLE_ROBOT_DATA_MODEL_DIR);

namespace pin = pinocchio;


int main(){

    const std::string urdf_path = path_to_all_urdf + "/talos_data/robots/talos_reduced_corrected.urdf";
    
    pin::Model model;
    pin::urdf::buildModel(urdf_path, model);
    pin::Data data(model);

    Eigen::VectorXd q = pin::randomConfiguration(model); //model.referenceConfigurations["half_sitting"];
    std::cout << "\n\n" << q << "\n\n" << std::endl;

    /*
    int i; i=0;
    while(i<20){  
        std::cout << "\n\n"<< i << model.jointPlacements[2].translation() << "\n\n" << std::endl;
        i++;
    }*/
}

