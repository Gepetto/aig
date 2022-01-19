#include<iostream>
#include<Eigen/Dense>
#include"../include/preview_IK/postures.hpp"

#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
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

    namespace pin = pinocchio;
    pin::Model model;

    legJoints solve_leg(const Homogeneous com, const Homogeneous foot, const Side side){

        if(side == LEFT){




        }
    }
}

int main(int argc, char *argv[]){

    IK_tools::HomMatrix m;
    m<<1, 2, 3, 4, 1, 2, 3, 4 ,1, 2, 3, 4, 12, 2, 3, 4;

    IK_tools::Homogeneous M(m);
    IK_tools::Side j = IK_tools::Side::RIGHT;

    std::cout << j << std::endl;


}