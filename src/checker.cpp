#include<iostream>
#include<Eigen/Dense>
#include<string.h>
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

#include<iostream>

// CHECKING FUNCTIONS
void load_talos_model(){

    pin::Model model;
    pin::urdf::buildModel(conf::urdf_path, pin::JointModelFreeFlyer(), model);
    std::cout << "\n\n" << model.name << "\n\n" << std::endl;
}

void get_referenceConfig(){

    pin::Model model;
    pin::urdf::buildModel(conf::urdf_path, pin::JointModelFreeFlyer(), model);
    pin::srdf::loadReferenceConfigurations(model, conf::srdf_path, false);
    Eigen::VectorXd q = model.referenceConfigurations["half_sitting"];
    std::cout << "\n\n q.size() : " << q.size() << "\n\n" << std::endl;
    std::cout << "\n\n model.nq : " << model.nq << "\n\n" << std::endl;
}

void compute_jointPlacement(){

    pin::Model model;
    pin::urdf::buildModel(conf::urdf_path, pin::JointModelFreeFlyer(), model);
    pin::srdf::loadReferenceConfigurations(model, conf::srdf_path, false);
    Eigen::VectorXd q = model.referenceConfigurations["half_sitting"];

    std::cout << "\n\n" << model.jointPlacements[2].translation() << "\n\n" << std::endl;
}

void check_CoM(){

    pin::Model model;
    pin::urdf::buildModel(conf::urdf_path, pin::JointModelFreeFlyer(), model);
    pin::srdf::loadReferenceConfigurations(model, conf::srdf_path, false);
    Eigen::VectorXd q0 = model.referenceConfigurations["half_sitting"];
    pin::Data data(model);
    Eigen::Vector3d c = pin::centerOfMass(model, data, q0);

    std::cout << "\n\n" << c << "\n\n" << std::endl;
}

void check_eigen_norms(){

    Eigen::Vector3d x(3, 4, 0);
    std::cout << "\n\n" << x.norm() << "\n\n" << std::endl;

}

void check_SE3(){

    pin::SE3 H;
    H.setIdentity();
    std::cout << "\n\n" << H.rotation() << "\n\n" << std::endl;
    std::cout << "\n\n" << H.translation() << "\n\n" << std::endl;
}

void check_BipIKConstructor() {

    IK_tools::BipIK H;
    std::cout << "\n\n" << H.info.leftHipJoint << "\n\n" << std::endl;
    std::cout << "\n\n" << H.info.comFromWaist << "\n\n" << std::endl;
    
    IK_tools::BipIK E(IK_tools::xyzVector(1, 1, 0));
    std::cout << "\n\n" << E.info.leftHipJoint << "\n\n" << std::endl;
    std::cout << "\n\n" << E.info.comFromWaist << "\n\n" << std::endl;

    IK_tools::BipedSettings conf;
    conf.leftHipJoint = conf::leftHipJointName;
    conf.rightHipJoint = conf::rightHipJointName;
    conf.rightKneeJoint = conf::rightKneeJointName;
    conf.leftKneeJoint = conf::leftKneeJointName;
    conf.leftAnkleJoint = conf::leftAnkleJointName;
    conf.rightAnkleJoint = conf::rightAnkleJointName;
    conf.leftFootFrame = conf::leftFootFrameName;
    conf.rightFootFrame = conf::rightFootFrameName;
    conf.comFromWaist = IK_tools::xyzVector(1, 1, 0);

    IK_tools::BipIK G(conf);
    std::cout << "\n\n" << G.info.model.name << "\n\n" << std::endl;
    std::cout << "\n\n" << G.info.comFromWaist << "\n\n" << std::endl;
    std::cout << "\n\n" << G.info.leftHipJoint << "\n\n" << std::endl;
}

void check_LegIGConstructor(){

    IK_tools::LegIG L;
    std::cout << "\n\n" << L.info.side << "\n\n" << std::endl;
    
    IK_tools::LegSettings conf;
    conf.side = IK_tools::Side::LEFT;
    conf.femurLength = 3.0;
    conf.tibiaLenght = 1.32;
    conf.hipFromWaist << 0, 1, 0;
    conf.ankleFromFoot << 0, 0, 34;
    
    IK_tools::LegIG J(conf);
    std::cout << "\n\n" << J.info.side << "\n\n" << std::endl;
    std::cout << "\n\n" << J.info.hipFromWaist << "\n\n" << std::endl;

    IK_tools::BipIK H;
    IK_tools::LegIG U = H.leftLeg;
    H.configurateLegs();
    std::cout << "\n\n" << U.info.side << "\n\n" << std::endl;
    std::cout << "\n\n" << U.info.hipFromWaist << "\n\n" << std::endl;
    std::cout << "\n\n" << U.info.femurLength << "\n\n" << std::endl;
}

void checkSolveMethods(){

    IK_tools::BipIK H;
    H.configurateLegs();
    Eigen::VectorXd q0 = H.info.model.referenceConfigurations["half_sitting"];
    pin::Data data(H.info.model);
    std::cout << "\n\n" << "q0 : " << q0 << "\n\n" << std::endl;

    pin::forwardKinematics(H.info.model, data, q0);
    pin::updateFramePlacements(H.info.model, data);
    pin::FrameIndex LF_id = H.info.model.getFrameId(conf::leftFootFrameName),
                    RF_id = H.info.model.getFrameId(conf::rightFootFrameName);

    pin::SE3  leftFoot(data.oMf[LF_id]), 
             rightFoot(data.oMf[RF_id]);
    std::cout << "\n\n" << "LF.translation : " << leftFoot.translation() << "\n\n" << std::endl;
    std::cout << "\n\n" << "RF.translation : " << rightFoot.translation() << "\n\n" << std::endl;
    IK_tools::xyzVector com(q0.head(3) + H.info.comFromWaist);
    
    pin::SE3 base = H.computeBase(com, leftFoot, rightFoot);
    std::cout << "\n\n" << "base.rotation : " << base.rotation() << "\n\n" << std::endl;
    std::cout << "\n\n" << "base.translation : " << base.translation() << "\n\n" << std::endl;

    std::cout << "\n\n" << "HipFromWaist : " << H.leftLeg.info.hipFromWaist << "\n\n" << std::endl;
    std::cout << "\n\n" << "HipFromWaist : " << H.rightLeg.info.hipFromWaist << "\n\n" << std::endl;
    std::cout << "\n\n" << "AnkleFromFoot : " << H.leftLeg.info.ankleFromFoot << "\n\n" << std::endl;
    std::cout << "\n\n" << "AnkleFromFoot : " << H.rightLeg.info.ankleFromFoot << "\n\n" << std::endl;
    std::cout << "\n\n" << "femur : " << H.leftLeg.info.femurLength << "\n\n" << std::endl;
    std::cout << "\n\n" << "femur : " << H.rightLeg.info.femurLength << "\n\n" << std::endl;
    std::cout << "\n\n" << "tibia : " << H.leftLeg.info.tibiaLenght << "\n\n" << std::endl;
    std::cout << "\n\n" << "tibia : " << H.rightLeg.info.tibiaLenght << "\n\n" << std::endl;

    IK_tools::legJoints L = H.leftLeg.solve(base, leftFoot);
    IK_tools::legJoints R = H.rightLeg.solve(base, rightFoot);

    std::cout << "\n\n" << "left leg joints : " << L << "\n\n" << std::endl;
    std::cout << "\n\n" << "right leg joints : " << R << "\n\n" << std::endl;

    Eigen::VectorXd posture;
    H.solve(base, leftFoot, rightFoot, q0, posture);
    std::cout << "\n\n" << "BipIK.solve : " << posture << "\n\n" << std::endl;

}

void check_solveDerivatives(){

    IK_tools::BipIK H;
    H.configurateLegs();
    Eigen::VectorXd q0 = H.info.model.referenceConfigurations["half_sitting"];
    pin::Data data(H.info.model);
    std::cout << "\n\n" << "q0 : " << q0 << "\n\n" << std::endl;

    pin::forwardKinematics(H.info.model, data, q0);
    pin::updateFramePlacements(H.info.model, data);
    pin::FrameIndex LF_id = H.info.model.getFrameId(conf::leftFootFrameName),
                    RF_id = H.info.model.getFrameId(conf::rightFootFrameName);

    pin::SE3 leftFoot(data.oMf[LF_id]), rightFoot(data.oMf[RF_id]);
    IK_tools::xyzVector com1, com2, com3;
    com1 = q0.head(3) + H.info.comFromWaist;
    com2 = IK_tools::xyzVector(0, 0.01, -0.02) + com1;
    com3 = IK_tools::xyzVector(0, 0.01, -0.02) + com2;
    std::array<IK_tools::xyzVector, 3> coms = {com1, com2, com3};
    std::array<pin::SE3, 3> LFs = {leftFoot, leftFoot, leftFoot};
    std::array<pin::SE3, 3> RFs = {rightFoot, rightFoot, rightFoot};

    Eigen::VectorXd posture, velocity, acceleration;

    H.solve(coms, LFs, RFs, q0, posture, velocity, acceleration, 0.01);

    std::cout << "\nPosture : \n" << posture << "\n\n" << std::endl;
    std::cout << "\nVelocity : \n" << velocity << "\n\n" << std::endl;
    std::cout << "\nAcceleration : \n" << acceleration << "\n\n" << std::endl;
}


// OLD CODE: /////////////////////////////

void checkChasisConstructor(){

    IK_tools::BipedIK H;
    // expected H.comFromWaist for talos: -0.003163900014529,  0.001237384291204, -0.142588610107038
    std::cout << "\n\nH.comFromWaist : " << H.comFromWaist << "\n\n" << std::endl;

    //nq should be 39
    std::cout << "\n\nH.model.nq : " << H.model.nq << "\n\n" << std::endl;
}

void checkLegComputation(){

    IK_tools::legJoints leg;
    IK_tools::BipedIK H;
    pin::SE3 com, foot;
    pin::Data data(H.model);
    
    pin::forwardKinematics(H.model, data, H.q0);
    pin::updateFramePlacements(H.model, data);
    std::string LF_name("leg_left_sole_fix_joint");
    long unsigned int LF_id = H.model.getFrameId(LF_name);

    foot = data.oMf[LF_id];
    com.setIdentity();
    com.translation(IK_tools::xyzVector(H.q0.head(3) + H.comFromWaist));

    // with q0, com = [-0.003163900014529,  0.001237384291204,  0.876681389892962]
    // left  foot = [-0.008846952891378,  0.084817244088858, -0.000002022956703]
    std::cout << "\n\nCoM : " << com.translation() << std::endl;
    std::cout << "\n\nFoot : " << foot.translation() << std::endl;

    // and left leg = [-0., -0.001708 , -0.411354,  0.859395, -0.448041,  0.]
    leg = H.solveLeg(com, foot, IK_tools::Side::LEFT);
    std::cout << "\n\n" << leg << "\n\n" << std::endl;
}

void checkPostureComputation(){

    IK_tools::legJoints leg;
    IK_tools::BipedIK H;
    pin::Data data(H.model);

    pin::forwardKinematics(H.model, data, H.q0);
    pin::updateFramePlacements(H.model, data);
    std::string LF_name("leg_left_sole_fix_joint"), RF_name("leg_right_sole_fix_joint");
    long unsigned int LF_id = H.model.getFrameId(LF_name), RF_id = H.model.getFrameId(RF_name);

    pin::SE3 leftFoot(data.oMf[LF_id]), rightFoot(data.oMf[RF_id]);
    pin::SE3 com;
    com.translation(IK_tools::xyzVector(H.q0.head(3) + H.comFromWaist));
    H.setRootOrientation(com, leftFoot, rightFoot);

    // with q0, com = [-0.003163900014529,  0.001237384291204,  0.876681389892962]
    // left  foot = [-0.00884695,  0.0848172, -2.02296e-06]
    // right  foot = [-0.00884695,  -0.0851828, -2.02296e-06]
    std::cout << "\n\nCoM : " << com.translation() << std::endl;
    std::cout << "\n\nLeft : " << leftFoot.translation() << std::endl;
    std::cout << "\n\nRight : " << rightFoot.translation() << std::endl;

    H.q = H.computePosture(com, leftFoot, rightFoot, H.q0);

    std::cout << "\n\n" << H.q << "\n\n" << std::endl;
    // the biggest element in the difference H.q0 - q_new is smaller than 0.0018
    std::cout << "\n\n" << H.q0 - H.q << "\n\n" << std::endl;
}

void checkDerivativesComputation(){

    IK_tools::legJoints leg;
    IK_tools::BipedIK H;
    pin::Data data(H.model);

    pin::forwardKinematics(H.model, data, H.q0);
    pin::updateFramePlacements(H.model, data);
    std::string LF_name("leg_left_sole_fix_joint"), RF_name("leg_right_sole_fix_joint");
    long unsigned int LF_id = H.model.getFrameId(LF_name), RF_id = H.model.getFrameId(RF_name);

    pin::SE3 leftFoot(data.oMf[LF_id]), rightFoot(data.oMf[RF_id]);
    pin::SE3 com1, com2, com3;
    com1.translation(IK_tools::xyzVector(H.q0.head(3) + H.comFromWaist));
    com2.translation(IK_tools::xyzVector(0, 0.01, -0.02)+com1.translation());
    com3.translation(IK_tools::xyzVector(0, 0.01, -0.02)+com2.translation());
    H.setRootOrientation(com1, leftFoot, rightFoot);
    H.setRootOrientation(com2, leftFoot, rightFoot);
    H.setRootOrientation(com3, leftFoot, rightFoot);

    Eigen::VectorXd q1 = H.computePosture(com1, leftFoot, rightFoot, H.q0);
    Eigen::VectorXd q2 = H.computePosture(com2, leftFoot, rightFoot, H.q0);
    Eigen::VectorXd q3 = H.computePosture(com3, leftFoot, rightFoot, H.q0);

    H.computeJointDerivatives(q1, q2, q3, 0.01);

    std::cout << "\n\n" << H.q << "\n\n" << std::endl;
    std::cout << "\n\n" << H.q0 - H.q << "\n\n" << std::endl;
    std::cout << "\n\n" << H.v << "\n\n" << std::endl;
    std::cout << "\n\n" << H.a << "\n\n" << std::endl;
}


int main(){
    check_solveDerivatives();
}

