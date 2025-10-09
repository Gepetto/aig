#include <boost/python.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <eigenpy/eigenpy.hpp>

#include "aig/contact6d.hpp"
#include "aig/python.hpp"

namespace aig {
namespace python {
namespace bp = boost::python;

bp::dict get_settings(Contact6D &self) {
  bp::dict settings;
  Contact6DSettings conf = self.getSettings();
  settings["frame_name"] = conf.frame_name;
  settings["gu"] = conf.gu;
  settings["mu"] = conf.mu;
  settings["weights"] = conf.weights;
  settings["half_width"] = conf.half_width;
  settings["half_length"] = conf.half_length;

  return settings;
}

void set_weights(Contact6DSettings &self, Eigen::Matrix<double, 6, 1> w) {
  self.weights = w;
}

Eigen::Matrix<double, 6, 1> get_weights(Contact6DSettings &self) {
  return self.weights;
}

void exposeContact6D() {
  bp::class_<Contact6DSettings>("Contact6DSettings")
      .def_readwrite("frame_name", &Contact6DSettings::frame_name)
      .def_readwrite("gu", &Contact6DSettings::gu)
      .def_readwrite("mu", &Contact6DSettings::mu)
      .add_property("weights", &get_weights, &set_weights)
      .def_readwrite("half_width", &Contact6DSettings::half_width)
      .def_readwrite("half_length", &Contact6DSettings::half_length)
      .def("__repr__", &Contact6DSettings::to_string)
      .def("__eq__", &Contact6DSettings::operator==)
      .def("__ne__", &Contact6DSettings::operator!=);

  bp::class_<Contact6D>("Contact6D", bp::init<>())
      .def("initialize", &Contact6D::initialize, bp::args("self", "settings"))
      .def("get_settings", &get_settings, bp::args("self"))
      .def("set_mu", &Contact6D::setMu, bp::args("self", "mu"))
      .def("set_gu", &Contact6D::setGu, bp::args("self", "gu"))
      .def("set_force_weights", &Contact6D::setForceWeights,
           bp::args("self", "force_weights"))
      .def("set_torque_weights", &Contact6D::setTorqueWeights,
           bp::args("self", "torque_weights"))
      .def("set_surface_half_width", &Contact6D::setSurfaceHalfWidth,
           bp::args("self", "half_width"))
      .def("set_surface_half_length", &Contact6D::setSurfaceHalfLength,
           bp::args("self", "half_length"))
      .def("update_NE_matrix", &Contact6D::updateNewtonEuler,
           bp::args("self", "CoM", "oMf"))
      .def("uni_A", &Contact6D::uni_A,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("uni_b", &Contact6D::uni_b,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("reg_A", &Contact6D::reg_A,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("reg_b", &Contact6D::reg_b,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("fri_A", &Contact6D::fri_A,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("fri_b", &Contact6D::fri_b,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("NE_A", &Contact6D::NE_A,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("uni_rows", &Contact6D::uni_rows, bp::args("self"))
      .def("fri_rows", &Contact6D::fri_rows, bp::args("self"))
      .def("cols", &Contact6D::cols, bp::args("self"))
      .def("get_frame_id", &Contact6D::getFrameID, bp::args("self"))
      .def("toWorldForces", &Contact6D::toWorldForces, bp::args("self"))
      .def("toCoMForces", &Contact6D::toCoMForces, bp::args("self"))
      .def("set_frame_id", &Contact6D::setFrameID, bp::args("self"))
      .def("applyForce", &Contact6D::applyForce, bp::args("self", "force"))
      .def("appliedForce", &Contact6D::appliedForce,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("get_pose", &Contact6D::getPose,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"));
}
} // namespace python
} // namespace aig
