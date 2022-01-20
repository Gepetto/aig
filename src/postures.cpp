#include<iostream>
#include<Eigen/Dense>
#include"../include/preview_IK/postures.hpp"

#include "example-robot-data/path.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/rnea.hpp"


namespace IK_tools{
    Homogeneous::Homogeneous(const HomMatrix mat){
        matrix = mat;
    }
    Homogeneous::Homogeneous(){
        matrix = HomMatrix::Identity();
    }
    RotMatrix Homogeneous::rotation(){
        return matrix.block<3, 3>(0,0);
    }
    xyzVector Homogeneous::translation(){
        return matrix.block<3, 1>(0, 3);
    }
    void Homogeneous::setRotation(const RotMatrix rotation){
        matrix.block<3, 3>(0,0)= rotation;
    }
    void Homogeneous::setTranslation(const xyzVector translation){
        matrix.block<3, 1>(0, 3) = translation;
    }

    std::string path_to_all_urdf (EXAMPLE_ROBOT_DATA_MODEL_DIR);
    const std::string urdf_path = path_to_all_urdf + "/talos_data/robots/talos_reduced_corrected.urdf";

    namespace pin = pinocchio;
    pin::Model model;


    legJoints solve_leg(const Homogeneous com, const Homogeneous foot, const Side side){

        if(side == LEFT){
            xyzVector HipFromWaist = model.jointPlacements[2].translation();




        }
    }
}

int main(){

    IK_tools::HomMatrix m;
    m<<1, 2, 3, 4, 1, 2, 3, 4 ,1, 2, 3, 4, 12, 2, 3, 4;

    IK_tools::Homogeneous M(m);
    IK_tools::Side j = IK_tools::Side::RIGHT;

    std::cout << j << std::endl;
}