#include<iostream>
#include<math.h>
#include<Eigen/Dense>
#include"../include/preview_IK/postures.hpp"
#include"../include/preview_IK/configuration.hpp"

#include "example-robot-data/path.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/frames.hpp"


namespace IK_tools{
    Homogeneous::Homogeneous(const HomMatrix mat){//DEPRECATED USE pin::SE3
        matrix = mat;
    }
    Homogeneous::Homogeneous(){
        matrix = HomMatrix::Identity();
    }
    RotMatrix Homogeneous::rotation() const{
        return matrix.block<3, 3>(0,0);
    }
    xyzVector Homogeneous::translation() const{
        return matrix.block<3, 1>(0, 3);
    }
    void Homogeneous::rotation(const RotMatrix rot){
        matrix.block<3, 3>(0,0)= rot;
    }
    void Homogeneous::translation(const xyzVector tran){
        matrix.block<3, 1>(0, 3) = tran;
    }

    Chassis::Chassis(){

        pin::urdf::buildModel(conf::urdf_path, pin::JointModelFreeFlyer(), model);
        pin::srdf::loadReferenceConfigurations(model, conf::srdf_path, false);
        q0 = model.referenceConfigurations["half_sitting"];
        pin::Data data(model);
        xyzVector c = pin::centerOfMass(model, data, q0);
        comFromWaist = c - q0.head(3);
    }
    Eigen::VectorXd Chassis::computePosture(const Homogeneous &com, const Homogeneous &leftFoot,
                                             const Homogeneous &rightFoot, Eigen::VectorXd q) const{
        
        q.head(3) = com.translation() - com.rotation() * comFromWaist;
        q.segment(7, 6) = solveLeg(com, leftFoot, IK_tools::Side::LEFT);
        q.segment(13, 6) = solveLeg(com, rightFoot, IK_tools::Side::RIGHT);
        return q;
    }

    legJoints Chassis::solveLeg(const Homogeneous &com, const Homogeneous &foot, const Side &side) const{

        xyzVector hipFromWaist;
        xyzVector ankleFromFoot;
        double distanceA;
        double distanceB;

        if(side == LEFT){
            hipFromWaist = model.jointPlacements[2].translation();
            ankleFromFoot = -model.frames[15].placement.translation();
            distanceA = model.jointPlacements[5].translation().norm();
            distanceB = model.jointPlacements[6].translation().norm();
        }else{
            hipFromWaist = model.jointPlacements[8].translation();
            ankleFromFoot = -model.frames[29].placement.translation();
            distanceA = model.jointPlacements[11].translation().norm();
            distanceB = model.jointPlacements[12].translation().norm();
        }
        xyzVector hip = com.translation() + com.rotation() * (hipFromWaist - comFromWaist);
        xyzVector ankle = foot.translation() + foot.rotation() * (ankleFromFoot);
        xyzVector hipFromAnkle = hip - ankle;
        double distanceC = hipFromAnkle.norm();

        double q2, q3, q4, q5, q6, q7;
        double cos_q5 = (pow(distanceC, 2) - pow(distanceA, 2) - pow(distanceB, 2))/(2.0 * distanceA * distanceB);
        
        if(cos_q5 >=1) q5 = 0;
        else if(cos_q5 <= -1) q5 = M_PI;
        else q5 = acos(cos_q5);

        q7 = atan2(hipFromAnkle(1), hipFromAnkle(2));
        if(q7 > M_PI_2) q7 -= M_PI;
        else if(q7 < -M_PI_2) q7 += M_PI;

        double zDirectionLeg;
        hipFromAnkle(2)>0? zDirectionLeg = 1 : zDirectionLeg = -1;
        q6 = -atan2(hipFromAnkle(0), zDirectionLeg * hipFromAnkle.tail<2>().norm()) - asin((distanceA/distanceC) * sin(M_PI - q5));
        
        IK_tools::RotMatrix Rint, Rext, R;
        Rext = com.rotation().transpose() * foot.rotation();
        Rint = Eigen::AngleAxisd(-q7, xyzVector(1, 0, 0)) * Eigen::AngleAxisd(-q5 - q6, xyzVector(0, 1, 0));
        R = Rext * Rint;
        q2 = atan2(-R(0, 1), R(1, 1));
        q3 = atan2(R(2, 1), -R(0, 1) * sin(q2) + R(1, 1) * cos(q2));
        q4 = atan2(-R(2, 0), R(2, 2));

        IK_tools::legJoints leg; leg << q2, q3, q4, q5, q6, q7;
        return leg;
    }
    //TODO: Remove the classe Homogeneous and use instead pin::SE3
}
    

/*
int main(){
    
}*/