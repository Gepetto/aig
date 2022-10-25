#include <pinocchio/multibody/fwd.hpp>

#include "aig/dyna_com.hpp"
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

Eigen::VectorXd solve(BipedIG &self, const Eigen::Vector3d &com,
                const pinocchio::SE3 &LF, const pinocchio::SE3 &RF,
                const Eigen::VectorXd &q0, const double &tolerance = 1e-10,
                const int &max_iterations = 0) {
  Eigen::VectorXd pos;
  self.solve(com, LF, RF, q0, pos, tolerance, max_iterations);
  return pos;
}
Eigen::VectorXd solve(BipedIG &self, const Eigen::Vector3d &com,
                const Eigen::Isometry3d &LF, const Eigen::Isometry3d &RF,
                const Eigen::VectorXd &q0, const double &tolerance = 1e-10,
                const int &max_iterations = 0) {
  Eigen::VectorXd pos;
  self.solve(com, LF, RF, q0, pos, tolerance, max_iterations);
  return pos;
}
Eigen::VectorXd solve(BipedIG &self, const Eigen::Vector3d &com,
                const Eigen::Matrix3d &baseRotation, const pinocchio::SE3 &LF,
                const pinocchio::SE3 &RF, const Eigen::VectorXd &q0,
                const double &tolerance = 1e-10,
                const int &max_iterations = 0) {
  Eigen::VectorXd pos;
  self.solve(com, baseRotation, LF, RF, q0, pos, tolerance, max_iterations);
  return pos;
}
Eigen::VectorXd solve(BipedIG &self, const Eigen::Vector3d &com,
                const Eigen::Matrix3d &baseRotation,
                const Eigen::Isometry3d &LF, const Eigen::Isometry3d &RF,
                const Eigen::VectorXd &q0, const double &tolerance = 1e-10,
                const int &max_iterations = 0) {
  Eigen::VectorXd pos;
  self.solve(com, baseRotation, LF, RF, q0, pos, tolerance, max_iterations);
  return pos;
}
Eigen::VectorXd solve(BipedIG &self, const pinocchio::SE3 &base,
                const pinocchio::SE3 &LF, const pinocchio::SE3 &RF,
                const Eigen::VectorXd &q0) {
  Eigen::VectorXd pos;
  self.solve(base, LF, RF, q0, pos);
  return pos;
}
Eigen::VectorXd solve(BipedIG &self, const Eigen::Isometry3d &base,
                const Eigen::Isometry3d &LF, const Eigen::Isometry3d &RF,
                const Eigen::VectorXd &q0) {
  Eigen::VectorXd pos;
  self.solve(base, LF, RF, q0, pos);
  return pos;
}

bp::tuple solve(BipedIG &self, const std::array<Eigen::Vector3d, 3> &com,
                const std::array<pinocchio::SE3, 3> &LF,
                const std::array<pinocchio::SE3, 3> &RF,
                const Eigen::VectorXd &q0, const double &dt,
                const double &tolerance = 1e-10,
                const int &max_iterations = 0) {
  Eigen::VectorXd pos, vel, acc;
  self.solve(com, LF, RF, q0, pos, vel, acc, dt, tolerance, max_iterations);
  return bp::tuple(pos);
}
bp::tuple solve(BipedIG &self, const std::array<Eigen::Vector3d, 3> &com,
                const std::array<Eigen::Isometry3d, 3> &LF,
                const std::array<Eigen::Isometry3d, 3> &RF,
                const Eigen::VectorXd &q0, const double &dt,
                const double &tolerance = 1e-10,
                const int &max_iterations = 0) {
  Eigen::VectorXd pos, vel, acc;
  self.solve(com, LF, RF, q0, pos, vel, acc, dt, tolerance, max_iterations);
  return bp::tuple(pos);
}
bp::tuple solve(BipedIG &self, const std::array<Eigen::Vector3d, 3> &com,
                const std::array<Eigen::Matrix3d, 3> &baseRotation,
                const std::array<pinocchio::SE3, 3> &LF,
                const std::array<pinocchio::SE3, 3> &RF,
                const Eigen::VectorXd &q0, const double &dt,
                const double &tolerance = 1e-10,
                const int &max_iterations = 0) {
  Eigen::VectorXd pos, vel, acc;
  self.solve(com, baseRotation, LF, RF, q0, pos, vel, acc, dt, tolerance,
             max_iterations);
  return bp::tuple(pos);
}
bp::tuple solve(BipedIG &self, const std::array<Eigen::Vector3d, 3> &com,
                const std::array<Eigen::Matrix3d, 3> &baseRotation,
                const std::array<Eigen::Isometry3d, 3> &LF,
                const std::array<Eigen::Isometry3d, 3> &RF,
                const Eigen::VectorXd &q0, const double &dt,
                const double &tolerance = 1e-10,
                const int &max_iterations = 0) {
  Eigen::VectorXd pos, vel, acc;
  self.solve(com, baseRotation, LF, RF, q0, pos, vel, acc, dt, tolerance,
             max_iterations);
  return bp::tuple(pos);
}
bp::tuple solve(BipedIG &self, const std::array<pinocchio::SE3, 3> &base,
                const std::array<pinocchio::SE3, 3> &LF,
                const std::array<pinocchio::SE3, 3> &RF,
                const Eigen::VectorXd &q0, const double &dt) {
  Eigen::VectorXd pos, vel, acc;
  self.solve(base, LF, RF, q0, pos, vel, acc, dt);
  return bp::tuple(pos);
}
bp::tuple solve(BipedIG &self, const std::array<Eigen::Isometry3d, 3> &base,
                const std::array<Eigen::Isometry3d, 3> &LF,
                const std::array<Eigen::Isometry3d, 3> &RF,
                const Eigen::VectorXd &q0, const double &dt) {
  Eigen::VectorXd pos, vel, acc;
  self.solve(base, LF, RF, q0, pos, vel, acc, dt);
  return bp::tuple(pos);
}

void exposeBiped_IG() {
  bp::class_<BipedIGSettings>("BipedIGSettings")
      .def_readwrite("left_hip_joint_name",
                     &BipedIGSettings::left_hip_joint_name)
      .def_readwrite("right_hip_joint_name",
                     &BipedIGSettings::right_hip_joint_name)
      .def_readwrite("left_knee_joint_name",
                     &BipedIGSettings::left_knee_joint_name)
      .def_readwrite("right_knee_joint_name",
                     &BipedIGSettings::right_knee_joint_name)
      .def_readwrite("left_ankle_joint_name",
                     &BipedIGSettings::left_ankle_joint_name)
      .def_readwrite("right_ankle_joint_name",
                     &BipedIGSettings::right_ankle_joint_name)
      .def_readwrite("left_foot_frame_name",
                     &BipedIGSettings::left_foot_frame_name)
      .def_readwrite("right_foot_frame_name",
                     &BipedIGSettings::right_foot_frame_name)
      .def("makeSettingsFor", &makeSettingsFor,
           bp::args("path_to_robots", "robot_name"))
      // .def("__eq__", &BipedIGSettings::operator==)
      // .def("__repr__", &BipedIGSettings::operator<<)
      ;

  bp::class_<LegIGSettings>("LegIGSettings")
     .def_readwrite("hip_from_waist", &LegIGSettings::hip_from_waist)
     .def_readwrite("knee_from_hip", &LegIGSettings::knee_from_hip)
     .def_readwrite("ankle_from_knee", &LegIGSettings::ankle_from_knee)
     .def_readwrite("ankle_from_foot", &LegIGSettings::ankle_from_foot);

  bp::class_<BipedIG>("BipedIG", bp::init<>())
      .def("initialize", &BipedIG::initialize, bp::args("self", "settings"))
      .def("get_settings", &BipedIG::get_settings,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("getQ0", &BipedIG::getQ0,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("setQ0", &BipedIG::setQ0, bp::args("self", "q0"))
      .def<Eigen::VectorXd (BipedIG &, const Eigen::Vector3d &, const pinocchio::SE3 &,
                     const pinocchio::SE3 &, const Eigen::VectorXd &,
                     const double &, const int &)>(
          "solve", &solve,
          (bp::args("self", "com", "leftFoot", "rightFoot", "q0"),
           bp::arg("tolerance") = 1e-10, bp::arg("max_iterations") = 0))
      .def<Eigen::VectorXd (BipedIG &, const Eigen::Vector3d &,
                     const Eigen::Isometry3d &, const Eigen::Isometry3d &,
                     const Eigen::VectorXd &, const double &, const int &)>(
          "solve", &solve,
          (bp::args("self", "com", "leftFoot", "rightFoot", "q0"),
           bp::arg("tolerance") = 1e-10, bp::arg("max_iterations") = 0))
      .def<Eigen::VectorXd (BipedIG &, const Eigen::Vector3d &,
                     const Eigen::Matrix3d &, const pinocchio::SE3 &,
                     const pinocchio::SE3 &, const Eigen::VectorXd &,
                     const double &, const int &)>(
          "solve", &solve,
          (bp::args("self", "com", "baseRotation", "leftFoot", "rightFoot",
                    "q0"),
           bp::arg("tolerance") = 1e-10, bp::arg("max_iterations") = 0))
      .def<Eigen::VectorXd (BipedIG &, const Eigen::Vector3d &,
                     const Eigen::Matrix3d &, const Eigen::Isometry3d &,
                     const Eigen::Isometry3d &, const Eigen::VectorXd &,
                     const double &, const int &)>(
          "solve", &solve,
          (bp::args("self", "com", "baseRotation", "leftFoot", "rightFoot",
                    "q0"),
           bp::arg("tolerance") = 1e-10, bp::arg("max_iterations") = 0))
      .def<Eigen::VectorXd (BipedIG &, const pinocchio::SE3 &, const pinocchio::SE3 &,
                     const pinocchio::SE3 &, const Eigen::VectorXd &)>(
          "solve", &solve,
          (bp::args("self", "base", "leftFoot", "rightFoot", "q0")))
      .def<Eigen::VectorXd (BipedIG &, const Eigen::Isometry3d &,
                     const Eigen::Isometry3d &, const Eigen::Isometry3d &,
                     const Eigen::VectorXd &)>(
          "solve", &solve,
          (bp::args("self", "base", "leftFoot", "rightFoot", "q0")))
      .def<bp::tuple(BipedIG &, const std::array<Eigen::Vector3d, 3> &,
                     const std::array<pinocchio::SE3, 3> &,
                     const std::array<pinocchio::SE3, 3> &,
                     const Eigen::VectorXd &, const double &, const double &,
                     const int &)>(
          "solve", &solve,
          (bp::args("self", "coms", "leftFeet", "rightFeet", "q0", "dt"),
           bp::arg("tolerance") = 1e-10, bp::arg("max_iterations") = 0))
      .def<bp::tuple(BipedIG &, const std::array<Eigen::Vector3d, 3> &,
                     const std::array<Eigen::Isometry3d, 3> &,
                     const std::array<Eigen::Isometry3d, 3> &,
                     const Eigen::VectorXd &, const double &, const double &,
                     const int &)>(
          "solve", &solve,
          (bp::args("self", "coms", "leftFeet", "rightFeet", "q0", "dt"),
           bp::arg("tolerance") = 1e-10, bp::arg("max_iterations") = 0))
      .def<bp::tuple(BipedIG &, const std::array<Eigen::Vector3d, 3> &,
                     const std::array<Eigen::Matrix3d, 3> &,
                     const std::array<pinocchio::SE3, 3> &,
                     const std::array<pinocchio::SE3, 3> &,
                     const Eigen::VectorXd &, const double &, const double &,
                     const int &)>(
          "solve", &solve,
          (bp::args("self", "coms",
                    "baseRotations"
                    "leftFeet",
                    "rightFeet", "q0", "dt"),
           bp::arg("tolerance") = 1e-10, bp::arg("max_iterations") = 0))
      .def<bp::tuple(BipedIG &, const std::array<Eigen::Vector3d, 3> &,
                     const std::array<Eigen::Matrix3d, 3> &,
                     const std::array<Eigen::Isometry3d, 3> &,
                     const std::array<Eigen::Isometry3d, 3> &,
                     const Eigen::VectorXd &, const double &, const double &,
                     const int &)>(
          "solve", &solve,
          (bp::args("self", "coms", "baseRotations", "leftFeet", "rightFeet",
                    "q0", "dt"),
           bp::arg("tolerance") = 1e-10, bp::arg("max_iterations") = 0))
      .def<bp::tuple(BipedIG &, const std::array<pinocchio::SE3, 3> &,
                     const std::array<pinocchio::SE3, 3> &,
                     const std::array<pinocchio::SE3, 3> &,
                     const Eigen::VectorXd &, const double &)>(
          "solve", &solve,
          (bp::args("self", "bases", "leftFeet", "rightFeet", "q0", "dt")))
      .def<bp::tuple(BipedIG &, const std::array<Eigen::Isometry3d, 3> &,
                     const std::array<Eigen::Isometry3d, 3> &,
                     const std::array<Eigen::Isometry3d, 3> &,
                     const Eigen::VectorXd &, const double &)>(
          "solve", &solve,
          (bp::args("self", "bases", "leftFeet", "rightFeet", "q0", "dt")))
      .def<void (BipedIG::*)(const Eigen::Vector3d &)>(
          "set_com_from_waist", &BipedIG::set_com_from_waist,
          bp::args("self", "com_from_waist"))
      .def<void (BipedIG::*)(const Eigen::VectorXd &)>(
          "set_com_from_waist", &BipedIG::set_com_from_waist,
          bp::args("self", "q"))
      .def("model", &BipedIG::get_model,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("data", &BipedIG::get_data,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def<void (BipedIG::*)(const Eigen::Vector3d &, const pinocchio::SE3 &,
                             const pinocchio::SE3 &, const Eigen::VectorXd &,
                             const double &, const int &)>(
          "correctCoMfromWaist", &BipedIG::correctCoMfromWaist,
          (bp::args("self", "com", "leftFoot", "rightFoot", "q0"),
           bp::arg("tolerance") = 1e-10, bp::arg("max_iterations") = 20))
      .def<void (BipedIG::*)(const Eigen::Vector3d &, const Eigen::Isometry3d &,
                             const Eigen::Isometry3d &, const Eigen::VectorXd &,
                             const double &, const int &)>(
          "correctCoMfromWaist", &BipedIG::correctCoMfromWaist,
          (bp::args("self", "com", "leftFoot", "rightFoot", "q0"),
           bp::arg("tolerance") = 1e-10, bp::arg("max_iterations") = 20))
      .def("get_com_from_waist", &BipedIG::get_com_from_waist,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("get_left_leg_settings", &BipedIG::get_left_leg_settings,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("get_right_leg_settings", &BipedIG::get_right_leg_settings,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      ;
}
}  // namespace python
}  // namespace aig
