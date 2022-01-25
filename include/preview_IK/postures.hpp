#ifndef __PREVIEW_IK_POSTURES___
#define __PREVIEW_IK_POSTURES___

#include<Eigen/Dense>
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/frames.hpp"

namespace pin = pinocchio;
namespace IK_tools {

    typedef Eigen::Matrix<double, 4, 4> HomMatrix;
    typedef Eigen::Matrix<double, 3, 3> RotMatrix;
    typedef Eigen::Matrix<double, 3, 1> xyzVector;
    typedef Eigen::Matrix<double, 6, 1> legJoints;
    enum Side {LEFT, RIGHT};
    
    class BipedIK
    {   
    public:
        xyzVector comFromWaist;
        pin::Model model;
        Eigen::VectorXd q0, q;
        Eigen::VectorXd v, a;
        BipedIK();
        legJoints solveLeg(const pin::SE3 &com, 
                            const pin::SE3 &foot, 
                            const Side &side) const;
        Eigen::VectorXd computePosture(const pin::SE3 &com, const pin::SE3 &leftFoot,
                                        const pin::SE3 &rightFoot, Eigen::VectorXd q);
        void setRootOrientation(pin::SE3 &com, const pin::SE3 &leftFoot, const pin::SE3 &rightFoot);
        void computeJointDerivatives(const Eigen::VectorXd &q1, 
                                                const Eigen::VectorXd &q2, 
                                                const Eigen::VectorXd &q3, const double dt);
    };


    struct LegSettings
    {
    public:
        Side side;
        double femurLength, tibiaLenght;
        xyzVector hipFromWaist, ankleFromFoot;
    };
    class LegIG
    {
    public:
        LegSettings info;

        LegIG();
        LegIG(const LegSettings &configuration);
        legJoints solve(const pin::SE3 &base, const pin::SE3 &endEffector);

    };
    

    class ArmIG
    {
    public:
        ArmIG();
        Eigen::VectorXd solve(const pin::SE3 &base, const pin::SE3 &endEffector);
    };

    struct BipedSettings
    {
    public:
        std::string leftHipJoint, leftKneeJoint, leftAnkleJoint, leftFootFrame,
                    rightHipJoint, rightKneeJoint, rightAnkleJoint, rightFootFrame;

        xyzVector comFromWaist;
        pin::Model model;
    };
    class BipIK
    {
    public:
        BipedSettings info;
        LegIG leftLeg, rightLeg;
        ArmIG leftArm, rightArm;

        BipIK();
        BipIK(const xyzVector &setComFromWaist);
        BipIK(const BipedSettings &configuration);
        void checkCompatibility();//TODO
        void solve(const xyzVector &com,
                   const pin::SE3 &leftFoot,
                   const pin::SE3 &rightFoot,
                   const Eigen::VectorXd &q0,
                   Eigen::VectorXd &posture);

        void solve(const xyzVector &com, 
                   const RotMatrix &baseRotation, 
                   const pin::SE3 &leftFoot, 
                   const pin::SE3 &rightFoot,
                   const Eigen::VectorXd &q0, 
                   Eigen::VectorXd &posture);

        void solve(const pin::SE3 &base,
                   const pin::SE3 &leftFoot,
                   const pin::SE3 &rightFoot,
                   const Eigen::VectorXd &q0,
                   Eigen::VectorXd &posture);

        void solve(const std::array<xyzVector, 3> &coms, 
                   const std::array<pin::SE3, 3> &leftFeet,
                   const std::array<pin::SE3, 3> &rightFeet,
                   const Eigen::VectorXd &q0, 
                   const double &dt,
                         Eigen::VectorXd &posture, 
                         Eigen::VectorXd &velocity, 
                         Eigen::VectorXd &acceleration);
        
        void solve(const std::array<xyzVector, 3> &coms, 
                   const std::array<RotMatrix, 3> &baseRotations,
                   const std::array<pin::SE3, 3> &leftFeet,
                   const std::array<pin::SE3, 3> &rightFeet,
                   const Eigen::VectorXd &q0, 
                   const double &dt,
                         Eigen::VectorXd &posture, 
                         Eigen::VectorXd &velocity, 
                         Eigen::VectorXd &acceleration);

        void solve(const std::array<pin::SE3, 3> &bases,
                   const std::array<pin::SE3, 3> &leftFeet,
                   const std::array<pin::SE3, 3> &rightFeet,
                   const Eigen::VectorXd &q0, 
                   const double &dt,
                         Eigen::VectorXd &posture, 
                         Eigen::VectorXd &velocity,
                         Eigen::VectorXd &acceleration); 

        void configurateLegs();
        pin::SE3 computeBase(const xyzVector &com,
                         const pin::SE3 &leftFoot,
                         const pin::SE3 &rightFoot);
        pin::SE3 computeBase(const xyzVector &com, 
                         const RotMatrix &baseRotation);
    };
}


#endif //__PREVIEW_IK_POSTURES___
