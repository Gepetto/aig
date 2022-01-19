#ifndef __PREVIEW_IK_POSTURES___
#define __PREVIEW_IK_POSTURES___

#include<Eigen/Dense>

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

        RotMatrix rotation();
        xyzVector translation();
        void setRotation(const RotMatrix rotation);
        void setTranslation(const xyzVector translation);
    };

    legJoints solve_leg(const Homogeneous com, const Homogeneous foot, const Side side);


}


#endif //__PREVIEW_IK_POSTURES___
