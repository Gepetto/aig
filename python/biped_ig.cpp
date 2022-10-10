#include "aig/dyna_com.hpp"

#include <pinocchio/multibody/fwd.hpp>  
// Must be included first!
#include <boost/python.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>

#include "aig/biped_ig.hpp"
#include "aig/python.hpp"

namespace aig {
  namespace python {
    namespace bp = boost::python;

    void exposeBiped_IG() {

      bp::class_<BipedIGSettings>("BipedIGSettings")
      .def_readwrite("left_hip_joint_name", &BipedIGSettings::left_hip_joint_name)
      .def_readwrite("right_hip_joint_name", &BipedIGSettings::right_hip_joint_name)
      .def_readwrite("left_knee_joint_name", &BipedIGSettings::left_knee_joint_name)
      .def_readwrite("right_knee_joint_name", &BipedIGSettings::right_knee_joint_name)
      .def_readwrite("left_ankle_joint_name", &BipedIGSettings::left_ankle_joint_name)
      .def_readwrite("right_ankle_joint_name", &BipedIGSettings::right_ankle_joint_name)
      .def_readwrite("left_foot_frame_name", &BipedIGSettings::left_foot_frame_name)
      .def_readwrite("right_foot_frame_name", &BipedIGSettings::right_foot_frame_name)
      .def("makeSettingsFor", &makeSettingsFor, bp::args("path_to_robots", "robot_name"))
      // .def("__eq__", &BipedIGSettings::operator==)
      // .def("__repr__", &BipedIGSettings::operator<<)
      ;

      bp::class_<BipedIG>("BipedIG", bp::init<>())
      .def("initialize", &BipedIG::initialize, bp::args("self", "settings"))
      .def("get_settings", &BipedIG::get_settings, bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
      .def("getQ0", &BipedIG::getQ0, bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
      .def("setQ0", &BipedIG::setQ0, bp::args("self", "q0"))
      .def<void (BipedIG::*)(const Eigen::Vector3d &, const pinocchio::SE3 &,
          const pinocchio::SE3 &, const Eigen::VectorXd &, Eigen::VectorXd &, 
          const double &, const int &)>
          ("solve", &BipedIG::solve, (bp::args("self", "com", "leftFoot", "rightFoot",
           "q0", "posture"), bp::arg("tolerance") = 1e-10, bp::arg("max_iterations")=0))
      .def<void (BipedIG::*)(const Eigen::Vector3d &, const Eigen::Isometry3d &,
          const Eigen::Isometry3d &, const Eigen::VectorXd &, Eigen::VectorXd &, 
          const double &, const int &)>
          ("solve", &BipedIG::solve, (bp::args("self", "com", "leftFoot", "rightFoot",
           "q0", "posture"), bp::arg("tolerance") = 1e-10, bp::arg("max_iterations")=0))
      .def<void (BipedIG::*)(const Eigen::Vector3d &, const Eigen::Matrix3d &, const pinocchio::SE3 &,
          const pinocchio::SE3 &, const Eigen::VectorXd &, Eigen::VectorXd &, 
          const double &, const int &)>
          ("solve", &BipedIG::solve, (bp::args("self", "com", "baseRotation", "leftFoot", "rightFoot",
           "q0", "posture"), bp::arg("tolerance") = 1e-10, bp::arg("max_iterations")=0))
      .def<void (BipedIG::*)(const Eigen::Vector3d &, const Eigen::Matrix3d &, const Eigen::Isometry3d &,
          const Eigen::Isometry3d &, const Eigen::VectorXd &, Eigen::VectorXd &, 
          const double &, const int &)>
          ("solve", &BipedIG::solve, (bp::args("self", "com", "baseRotation", "leftFoot", "rightFoot",
           "q0", "posture"), bp::arg("tolerance") = 1e-10, bp::arg("max_iterations")=0))
      .def<void (BipedIG::*)(const pinocchio::SE3 &, const pinocchio::SE3 &,
             const pinocchio::SE3 &, const Eigen::VectorXd &, Eigen::VectorXd &)>
          ("solve", &BipedIG::solve, (bp::args("self", "base", "leftFoot", "rightFoot",
           "q0", "posture")))
      .def<void (BipedIG::*)(const Eigen::Isometry3d &, const Eigen::Isometry3d &,
             const Eigen::Isometry3d &, const Eigen::VectorXd &, Eigen::VectorXd &)>
          ("solve", &BipedIG::solve, (bp::args("self", "base", "leftFoot", "rightFoot",
           "q0", "posture")))
      .def<void (BipedIG::*)(const std::array<Eigen::Vector3d, 3> &,
             const std::array<pinocchio::SE3, 3> &,
             const std::array<pinocchio::SE3, 3> &,
             const Eigen::VectorXd &, Eigen::VectorXd &,
             Eigen::VectorXd &, Eigen::VectorXd &,
             const double &, const double &,
             const int &)>
          ("solve", &BipedIG::solve, (bp::args("self", "coms", "leftFeet", "rightFeet",
           "q0", "posture", "velocity", "acceleration", "dt"), bp::arg("tolerance") = 1e-10, bp::arg("max_iterations")=0))
      .def<void (BipedIG::*)(const std::array<Eigen::Vector3d, 3> &,
             const std::array<Eigen::Isometry3d, 3> &,
             const std::array<Eigen::Isometry3d, 3> &,
             const Eigen::VectorXd &, Eigen::VectorXd &,
             Eigen::VectorXd &, Eigen::VectorXd &,
             const double &, const double &,
             const int &)>
          ("solve", &BipedIG::solve, (bp::args("self", "coms", "leftFeet", "rightFeet",
           "q0", "posture", "velocity", "acceleration", "dt"), bp::arg("tolerance") = 1e-10, bp::arg("max_iterations")=0))
      .def<void (BipedIG::*)(const std::array<Eigen::Vector3d, 3> &,
            const std::array<Eigen::Matrix3d, 3> &,
             const std::array<pinocchio::SE3, 3> &,
             const std::array<pinocchio::SE3, 3> &,
             const Eigen::VectorXd &, Eigen::VectorXd &,
             Eigen::VectorXd &, Eigen::VectorXd &,
             const double &, const double &,
             const int &)>
          ("solve", &BipedIG::solve, (bp::args("self", "coms", "baseRotations" "leftFeet", "rightFeet",
           "q0", "posture", "velocity", "acceleration", "dt"), bp::arg("tolerance") = 1e-10, bp::arg("max_iterations")=0))
      .def<void (BipedIG::*)(const std::array<Eigen::Vector3d, 3> &,
            const std::array<Eigen::Matrix3d, 3> &,
             const std::array<Eigen::Isometry3d, 3> &,
             const std::array<Eigen::Isometry3d, 3> &,
             const Eigen::VectorXd &, Eigen::VectorXd &,
             Eigen::VectorXd &, Eigen::VectorXd &,
             const double &, const double &,
             const int &)>
          ("solve", &BipedIG::solve, (bp::args("self", "coms", "baseRotations", "leftFeet", "rightFeet",
           "q0", "posture", "velocity", "acceleration", "dt"), bp::arg("tolerance") = 1e-10, bp::arg("max_iterations")=0))
      .def<void (BipedIG::*)(const std::array<pinocchio::SE3, 3> &,
             const std::array<pinocchio::SE3, 3> &,
             const std::array<pinocchio::SE3, 3> &,
             const Eigen::VectorXd &, Eigen::VectorXd &,
             Eigen::VectorXd &, Eigen::VectorXd &,
             const double &)>
          ("solve", &BipedIG::solve, (bp::args("self", "bases", "leftFeet", "rightFeet",
           "q0", "posture", "velocity", "acceleration", "dt")))
      .def<void (BipedIG::*)(const std::array<Eigen::Isometry3d, 3> &,
             const std::array<Eigen::Isometry3d, 3> &,
             const std::array<Eigen::Isometry3d, 3> &,
             const Eigen::VectorXd &, Eigen::VectorXd &,
             Eigen::VectorXd &, Eigen::VectorXd &,
             const double &)>
          ("solve", &BipedIG::solve, (bp::args("self", "bases", "leftFeet", "rightFeet",
           "q0", "posture", "velocity", "acceleration", "dt")))
      .def<void (BipedIG::*)(const Eigen::Vector3d &)>("set_com_from_waist", &BipedIG::set_com_from_waist, bp::args("self", "com_from_waist"))
      .def<void (BipedIG::*)(const Eigen::VectorXd &)>("set_com_from_waist", &BipedIG::set_com_from_waist, bp::args("self", "q"))
      .def("model", &BipedIG::get_model, bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
      .def("data", &BipedIG::get_data, bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
      .def<void (BipedIG::*)(const Eigen::Vector3d &,const pinocchio::SE3 &,
                           const pinocchio::SE3 &,const Eigen::VectorXd &,
                           const double &, const int &)>
          ("correctCoMfromWaist", &BipedIG::correctCoMfromWaist, (bp::args("self",
           "com", "leftFoot", "rightFoot", "q0"), bp::arg("tolerance")=1e-10, 
           bp::arg("max_iterations")=20))
      .def<void (BipedIG::*)(const Eigen::Vector3d &,const Eigen::Isometry3d &,
                             const Eigen::Isometry3d &,const Eigen::VectorXd &,
                             const double &, const int &)>
          ("correctCoMfromWaist", &BipedIG::correctCoMfromWaist, (bp::args("self",
           "com", "leftFoot", "rightFoot", "q0"), bp::arg("tolerance")=1e-10, 
           bp::arg("max_iterations")=20))
      .def("get_com_from_waist", &BipedIG::get_com_from_waist, bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
      ;
    }

  }  // namespace
}
