#include<iostream>
#include<math.h>
#include<Eigen/Dense>
#include<Eigen/Geometry>
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
    
    BipIK::BipIK(){

        pin::urdf::buildModel(conf::urdf_path, pin::JointModelFreeFlyer(), info.model);
        info.leftHipJoint = conf::leftHipJointName;
        info.rightHipJoint = conf::rightHipJointName;
        info.leftKneeJoint = conf::leftKneeJointName;
        info.rightKneeJoint = conf::rightKneeJointName;
        info.leftAnkleJoint = conf::leftAnkleJointName;
        info.rightAnkleJoint = conf::rightAnkleJointName;
        info.leftFootFrame = conf::leftFootFrameName;
        info.rightFootFrame = conf::rightFootFrameName;

        pin::srdf::loadReferenceConfigurations(info.model, conf::srdf_path, false);
        Eigen::VectorXd q0 = info.model.referenceConfigurations["half_sitting"];
        pin::Data data(info.model);
        xyzVector c = pin::centerOfMass(info.model, data, q0);
        info.comFromWaist = c - q0.head(3);
    }
    BipIK::BipIK(const xyzVector &setComFromWaist){

        info.leftHipJoint = conf::leftHipJointName;
        info.rightHipJoint = conf::rightHipJointName;
        info.leftKneeJoint = conf::leftKneeJointName;
        info.rightKneeJoint = conf::rightKneeJointName;
        info.leftAnkleJoint = conf::leftAnkleJointName;
        info.rightAnkleJoint = conf::rightAnkleJointName;
        info.leftFootFrame = conf::leftFootFrameName;
        info.rightFootFrame = conf::rightFootFrameName;
        info.comFromWaist = setComFromWaist;
        pin::urdf::buildModel(conf::urdf_path, pin::JointModelFreeFlyer(), info.model);
    }
    BipIK::BipIK(const BipedSettings &configuration){
        info = configuration;
        pin::urdf::buildModel(conf::urdf_path, pin::JointModelFreeFlyer(), info.model);
    }
    void BipIK::configurateLegs(){
        LegSettings leftConf, rightConf;
        pin::JointIndex leftHipID  = info.model.getJointId(info.leftHipJoint), 
                        leftKneeID = info.model.getJointId(info.leftKneeJoint), 
                        leftAnkleID  = info.model.getJointId(info.leftAnkleJoint),
                        rightHipID  = info.model.getJointId(info.rightHipJoint),
                        rightKneeID = info.model.getJointId(info.rightKneeJoint), 
                        rightAnkleID  = info.model.getJointId(info.rightAnkleJoint);
                        
        pin::FrameIndex leftSoleID = info.model.getFrameId(info.leftFootFrame),
                        rightSoleID = info.model.getFrameId(info.rightFootFrame);
        leftConf.side = Side::LEFT;
        leftConf.hipFromWaist = info.model.jointPlacements[leftHipID].translation();
        leftConf.femurLength = info.model.jointPlacements[leftKneeID].translation().norm();
        leftConf.tibiaLenght = info.model.jointPlacements[leftAnkleID].translation().norm();
        leftConf.ankleFromFoot = -info.model.frames[leftSoleID].placement.translation();
        leftLeg.info = leftConf;

        rightConf.side = Side::RIGHT;
        rightConf.hipFromWaist = info.model.jointPlacements[rightHipID].translation();
        rightConf.femurLength = info.model.jointPlacements[rightKneeID].translation().norm();
        rightConf.tibiaLenght = info.model.jointPlacements[rightAnkleID].translation().norm();
        rightConf.ankleFromFoot = -info.model.frames[rightSoleID].placement.translation();
        rightLeg.info = rightConf;
    }
    pin::SE3 BipIK::computeBase(const xyzVector &com, const pin::SE3 &leftFoot, const pin::SE3 &rightFoot){
        pin::SE3 base;
        double leftYawl, rightYawl, baseYawl;
        leftYawl = pin::log3(leftFoot.rotation())(2);
        rightYawl = pin::log3(rightFoot.rotation())(2);

        baseYawl = (leftYawl + rightYawl)/2;

        base.rotation(Eigen::AngleAxisd(baseYawl, xyzVector(0, 0, 1)).toRotationMatrix());
        base.translation(com - base.rotation() * info.comFromWaist);
        return base;
    }
    pin::SE3 BipIK::computeBase(const xyzVector &com, const RotMatrix &baseRotation){
        pin::SE3 base;

        base.rotation(baseRotation);
        base.translation(com - base.rotation() * info.comFromWaist);
        return base;
    }

    void BipIK::solve(const xyzVector &com, const pin::SE3 &leftFoot, const pin::SE3 &rightFoot,
                      const Eigen::VectorXd &q0, Eigen::VectorXd &posture){
        
        posture = q0;
        pin::SE3 base = computeBase(com, leftFoot, rightFoot);
        posture.head(3) = base.translation();
        Eigen::Quaterniond quat(base.rotation());
        posture.segment(3, 4) << quat.x(), quat.y(), quat.z(), quat.w();
        posture.segment(7, 6) = leftLeg.solve(base, leftFoot);
        posture.segment(13, 6) = rightLeg.solve(base, rightFoot);
        //TODO remove hard coded numbers 7, 6, 13, 6.
    }
    void BipIK::solve(const xyzVector &com, const RotMatrix &baseRotation, const pin::SE3 &leftFoot, const pin::SE3 &rightFoot,
                      const Eigen::VectorXd &q0, Eigen::VectorXd &posture){
        
        posture = q0;
        pin::SE3 base = computeBase(com, baseRotation);
        Eigen::Quaterniond quat(base.rotation());
        posture.segment(3, 4) << quat.x(), quat.y(), quat.z(), quat.w();
        posture.segment(7, 6) = leftLeg.solve(base, leftFoot);
        posture.segment(13, 6) = rightLeg.solve(base, rightFoot);
        //TODO remove hard coded numbers 7, 6, 13, 6.
    }
    void BipIK::solve(const pin::SE3 &base, const pin::SE3 &leftFoot, const pin::SE3 &rightFoot,
               const Eigen::VectorXd &q0, Eigen::VectorXd &posture){
        
        posture = q0;
        posture.head(3) = base.translation();
        Eigen::Quaterniond quat(base.rotation());
        posture.segment(3, 4) << quat.x(), quat.y(), quat.z(), quat.w();
        posture.segment(7, 6) = leftLeg.solve(base, leftFoot);
        posture.segment(13, 6) = rightLeg.solve(base, rightFoot);
        //TODO the numbers 7, 6, 13, 6 could be not hard coded.
    }
    void BipIK::solve(const std::array<xyzVector, 3> &coms, const std::array<pin::SE3, 3> &leftFeet,
                      const std::array<pin::SE3, 3> &rightFeet, const Eigen::VectorXd &q0, Eigen::VectorXd &posture,
                      Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration, const double &dt){
        
        Eigen::VectorXd q1, q3;
        solve(coms[0], leftFeet[0], rightFeet[0], q0, q1);
        solve(coms[1], leftFeet[1], rightFeet[1], q0, posture);
        solve(coms[2], leftFeet[2], rightFeet[2], q0, q3);

        Eigen::VectorXd velocity1(pin::difference(info.model, q1, posture)/dt);
        Eigen::VectorXd velocity3(pin::difference(info.model, posture, q3)/dt);

        velocity = pin::difference(info.model, q1, q3)/(2*dt);
        acceleration = (velocity3-velocity1)/dt;
    }
    void BipIK::solve(const std::array<xyzVector, 3> &coms, const std::array<RotMatrix, 3> &baseRotations,
                      const std::array<pin::SE3, 3> &leftFeet, const std::array<pin::SE3, 3> &rightFeet, 
                      const Eigen::VectorXd &q0, Eigen::VectorXd &posture, Eigen::VectorXd &velocity, 
                      Eigen::VectorXd &acceleration, const double &dt){
        
        Eigen::VectorXd q1, q3;
        solve(coms[0], baseRotations[0], leftFeet[0], rightFeet[0], q0, q1);
        solve(coms[1], baseRotations[1], leftFeet[1], rightFeet[1], q0, posture);
        solve(coms[2], baseRotations[2], leftFeet[2], rightFeet[2], q0, q3);

        Eigen::VectorXd velocity1(pin::difference(info.model, q1, posture)/dt);
        Eigen::VectorXd velocity3(pin::difference(info.model, posture, q3)/dt);

        velocity = pin::difference(info.model, q1, q3)/(2*dt);
        acceleration = (velocity3-velocity1)/dt;
    }
    void BipIK::solve(const std::array<pin::SE3, 3> &bases, const std::array<pin::SE3, 3> &leftFeet, 
                      const std::array<pin::SE3, 3> &rightFeet, const Eigen::VectorXd &q0, Eigen::VectorXd &posture, 
                      Eigen::VectorXd &velocity, Eigen::VectorXd &acceleration, const double &dt){
        
        Eigen::VectorXd q1, q3;
        solve(bases[0], leftFeet[0], rightFeet[0], q0, q1);
        solve(bases[1], leftFeet[1], rightFeet[1], q0, posture);
        solve(bases[2], leftFeet[2], rightFeet[2], q0, q3);

        Eigen::VectorXd velocity1(pin::difference(info.model, q1, posture)/dt);
        Eigen::VectorXd velocity3(pin::difference(info.model, posture, q3)/dt);

        velocity = pin::difference(info.model, q1, q3)/(2*dt);
        acceleration = (velocity3-velocity1)/dt;
    }

    Eigen::Vector2d BipIK::computeCoP(pin::Data &data, const Eigen::VectorXd &posture, 
                                      const Eigen::VectorXd &velocity, const Eigen::VectorXd &acceleration,
                                      bool flatHorizontalGround){

        Wrench tauMw = pin::rnea(info.model, data, posture, velocity, acceleration).head(6);
        Eigen::Vector3d groundTorqueMo = tauMw.tail(3) + 
                                         pin::skew(Eigen::Vector3d(posture.head(3)))*tauMw.head(3);
        std::cout << "groundTorque: \n" << groundTorqueMo << std::endl;
        Eigen::Vector3d pressureTorqueMo; 
        if(flatHorizontalGround){
            pressureTorqueMo = groundTorqueMo;
        }else{
            // TODO get the force distribution and remove the non pressure terms form the CoP computation.
            // for now, we assume a flat and horizontal ground.
        }
        Eigen::Vector2d cop;
        cop << -pressureTorqueMo(1)/tauMw(2), 
                pressureTorqueMo(0)/tauMw(2);
        return cop;
    }
 
    Eigen::Vector2d BipIK::computeCoP(pin::Data &data, const Eigen::VectorXd &posture, 
                                      const Eigen::VectorXd &velocity, const Eigen::VectorXd &acceleration,
                                      const Wrench &externalWrench, bool flatHorizontalGround){
        // The external wrench is suposed to be expresed in the frame of the root link.

        Wrench tauMw = pin::rnea(info.model, data, posture, velocity, acceleration).head(6);
        Eigen::Vector3d groundTorqueMo = tauMw.tail(3) - externalWrench.tail(3) + 
                                         pin::skew(Eigen::Vector3d(posture.head(3)))*(tauMw.head(3) - externalWrench.head(3));

        Eigen::Vector3d pressureTorqueMo;
        if(flatHorizontalGround){
            pressureTorqueMo = groundTorqueMo;
        }else{
            // TODO get the force distribution and remove the non pressure terms form the CoP computation.
            // for now, we assume a flat and horizontal ground.
        }
        Eigen::Vector2d cop;
        cop << -pressureTorqueMo(1)/(tauMw(2) - externalWrench(2)),
                pressureTorqueMo(0)/(tauMw(2) - externalWrench(2));
        return cop;
    }

    LegIG::LegIG(){}
    LegIG::LegIG(const LegSettings &configuration){
        info = configuration;
    }
    legJoints LegIG::solve(const pin::SE3 &base, const pin::SE3 &endEffector){

        xyzVector hip = base.translation() + base.rotation() * info.hipFromWaist;
        xyzVector ankle = endEffector.translation() + endEffector.rotation() * info.ankleFromFoot;
        xyzVector hipFromAnkle = hip - ankle;
        double distanceHipAnkle = hipFromAnkle.norm();

        double q2, q3, q4, q5, q6, q7;
        double cos_q5 = (pow(distanceHipAnkle, 2) - 
                         pow(info.femurLength, 2) - 
                         pow(info.tibiaLenght, 2))/(2.0 * info.femurLength * 
                                                          info.tibiaLenght);
        if(cos_q5 >=1) q5 = 0;
        else if(cos_q5 <= -1) q5 = M_PI;
        else q5 = acos(cos_q5);

        q7 = atan2(hipFromAnkle(1), hipFromAnkle(2));
        if(q7 > M_PI_2) q7 -= M_PI;
        else if(q7 < -M_PI_2) q7 += M_PI;

        int zDirectionLeg;
        hipFromAnkle(2)>0? zDirectionLeg = 1 : zDirectionLeg = -1;

        q6 = -atan2(hipFromAnkle(0), 
                    zDirectionLeg * hipFromAnkle.tail<2>().norm()) - 
              asin((info.femurLength/distanceHipAnkle) * sin(M_PI - q5));
        
        IK_tools::RotMatrix Rint, Rext, R;
        Rext = base.rotation().transpose() * endEffector.rotation();
        Rint = Eigen::AngleAxisd(-q7, xyzVector(1, 0, 0)) * Eigen::AngleAxisd(-q5 - q6, xyzVector(0, 1, 0));
        R = Rext * Rint;
        q2 = atan2(-R(0, 1), R(1, 1));
        q3 = atan2(R(2, 1), -R(0, 1) * sin(q2) + R(1, 1) * cos(q2));
        q4 = atan2(-R(2, 0), R(2, 2));

        IK_tools::legJoints leg; leg << q2, q3, q4, q5, q6, q7;
        return leg;
    }

    ArmIG::ArmIG() {}
    
    //OLD CODE: //////////////////////////////////////////////////////////////////////////////

    BipedIK::BipedIK(){

        pin::urdf::buildModel(conf::urdf_path, pin::JointModelFreeFlyer(), model);
        pin::srdf::loadReferenceConfigurations(model, conf::srdf_path, false);
        q0 = model.referenceConfigurations["half_sitting"];
        pin::Data data(model);
        xyzVector c = pin::centerOfMass(model, data, q0);
        comFromWaist = c - q0.head(3);
    }
    
    void BipedIK::setRootOrientation(pin::SE3 &com, const pin::SE3 &leftFoot, const pin::SE3 &rightFoot){

        double leftYawl, rightYawl, comYawl;
        leftYawl = pin::log3(leftFoot.rotation())(2);
        rightYawl = pin::log3(rightFoot.rotation())(2);

        comYawl = (leftYawl + rightYawl)/2;
        IK_tools::RotMatrix comR =Eigen::AngleAxisd(comYawl, xyzVector(0, 0, 1)).toRotationMatrix();
        com.rotation(comR);
    }

    legJoints BipedIK::solveLeg(const pin::SE3 &com, const pin::SE3 &foot, const Side &side) const{

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

    Eigen::VectorXd BipedIK::computePosture(const pin::SE3 &com, const pin::SE3 &leftFoot,
                                             const pin::SE3 &rightFoot, Eigen::VectorXd posture){
        
        posture.head(3) = com.translation() - com.rotation() * comFromWaist;
        posture.segment(7, 6) = solveLeg(com, leftFoot, IK_tools::Side::LEFT);
        posture.segment(13, 6) = solveLeg(com, rightFoot, IK_tools::Side::RIGHT);
        return posture; 
    }

    void BipedIK::computeJointDerivatives(const Eigen::VectorXd &posture1, 
                                          const Eigen::VectorXd &posture2, 
                                          const Eigen::VectorXd &posture3, const double dt){

        Eigen::VectorXd velocity1(pin::difference(model, posture1, posture2)/dt);
        Eigen::VectorXd velocity2(pin::difference(model, posture1, posture3)/(2*dt));
        Eigen::VectorXd velocity3(pin::difference(model, posture2, posture3)/dt);

        Eigen::VectorXd acceleration2 = (velocity3-velocity1)/dt;
        
        q = posture2;
        v = velocity2;
        a = acceleration2;
    }





}
    

