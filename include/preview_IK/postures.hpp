#ifndef __PREVIEW_IK_POSTURES___
#define __PREVIEW_IK_POSTURES___

#include<Eigen/Dense>
#include "pinocchio/parsers/sample-models.hpp"

namespace pin = pinocchio;
namespace IK_tools {

    typedef Eigen::Matrix<double, 4, 4> HomMatrix;
    typedef Eigen::Matrix<double, 3, 3> RotMatrix;
    typedef Eigen::Matrix<double, 3, 1> xyzVector;
    typedef Eigen::Matrix<double, 6, 1> legJoints;
    enum Side {LEFT, RIGHT};

    class Homogeneous
    {
    public:
        HomMatrix matrix;
        Homogeneous();
        Homogeneous(const HomMatrix mat);

        RotMatrix rotation() const;
        xyzVector translation() const;
        void rotation(const RotMatrix rot);
        void translation(const xyzVector tran);
    };

    class Horizon
    {
    public:
        Horizon();
    };

    class Chassis
    {   
    public:
        xyzVector comFromWaist;
        pin::Model model;
        Eigen::VectorXd q0;
        Chassis();
        legJoints solveLeg(const Homogeneous &com, 
                            const Homogeneous &foot, 
                            const Side &side) const;
        Eigen::VectorXd computePosture(const Homogeneous &com, const Homogeneous &leftFoot,
                                        const Homogeneous &rightFoot, Eigen::VectorXd q) const;
        void setGlobalOrientation(Homogeneous &com, const Homogeneous &leftFoot,
                                   const Homogeneous &rightFoot);
    };


}


#endif //__PREVIEW_IK_POSTURES___
