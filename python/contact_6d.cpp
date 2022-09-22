
#include <boost/python.hpp>
#include <boost/python/module.hpp>
#include <boost/python/return_internal_reference.hpp>
// #include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>

#include "aig/c_dynamics.hpp"
// #include "aig/python.hpp"

namespace aig {
namespace python {
namespace bp = boost::python;

void initialize(Contact6D &self, const bp::dict &settings) {
  Contact6DSettings conf;

  conf.active = bp::extract<bool>(settings["active"]);
  conf.frame_name = bp::extract<std::string>(settings["frame_name"]);
  conf.gu = bp::extract<double>(settings["gu"]);
  conf.mu = bp::extract<double>(settings["mu"]);
  conf.half_length = bp::extract<double>(settings["half_length"]);
  conf.half_width = bp::extract<bool>(settings["half_width"]);
  conf.weights = bp::extract<Eigen::Matrix<double, 6, 1>>(settings["active"]);

  self.initialize(conf);
}

bp::dict get_settings(Contact6D &self) {
  bp::dict settings;
  Contact6DSettings conf = self.getSettings();
  settings["frame_name"] = conf.frame_name;
  settings["gu"] = conf.gu;
  settings["mu"] = conf.mu;
  settings["active"] = conf.active;
  settings["weights"] = conf.weights;
  settings["half_width"] = conf.half_width;
  settings["half_length"] = conf.half_length;

  return settings;
}

  void exposeContact6DSettings() { 
     bp::class_<Contact6DSettings>("Contact6DSettings");
     bp::class_<DynoSettings>("DynoSettings");
     

     // bp::class_<Contact6D>("Cont");
  }


// void exposeContact6D() {
//   bp::class_<Contact6D, boost::noncopyable>("Contact6D", bp::init<>())
//       .def("initialize", initialize, bp::args("self", "settings"))
     //  .def("get_settings", get_settings, bp::args("self"))
     //  .def("set_active", &Contact6D::active, bp::args("self", "active"))
     //  .def("set_mu", &Contact6D::setMu, bp::args("self", "mu"))
     //  .def("set_gu", &Contact6D::setGu, bp::args("self", "gu"))
     //  .def("set_force_weights", &Contact6D::setForceWeights,
     //       bp::args("self", "force_weights"))
     //  .def("set_torque_weights", &Contact6D::setTorqueWeights,
     //       bp::args("self", "torque_weights"))
     //  .def("set_surface_half_width", &Contact6D::setSurfaceHalfWidth,
     //       bp::args("self", "half_width"))
     //  .def("set_surface_half_length", &Contact6D::setSurfaceHalfLength,
     //       bp::args("self", "half_length"))
     //  .def("update_NE_matrix", &Contact6D::updateNewtonEuler,
     //       bp::args("self", "CoM", "oMf"))
     //  .def("uni_A", &Contact6D::uni_A,bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
     //  .def("uni_b", &Contact6D::uni_b,bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
     //  .def("reg_A", &Contact6D::reg_A,bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
     //  .def("reg_b", &Contact6D::reg_b,bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
     //  .def("fri_A", &Contact6D::fri_A,bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
     //  .def("fri_b", &Contact6D::fri_b,bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
     //  .def("NE_A", &Contact6D::NE_A,bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
     //  .def("uni_rows", &Contact6D::uni_rows, bp::args("self"))
     //  .def("fri_rows", &Contact6D::fri_rows, bp::args("self"))
     //  .def("cols", &Contact6D::cols, bp::args("self"))
     //  .def("get_frame_id", &Contact6D::getFrameID, bp::args("self"))
     //  .def("toWorldForces", &Contact6D::toWorldForces,bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
     //  .def("set_frame_id", &Contact6D::setFrameID, bp::args("self"))
     //  .def("applyForce", &Contact6D::applyForce, bp::args("self", "force"))
     //  .def("appliedForce", &Contact6D::appliedForce,bp::return_value_policy<bp::reference_existing_object>(), bp::args("self"))
     // ;
// }
}  // namespace python
}  // namespace aig

BOOST_PYTHON_MODULE(aig) {
    // Enabling eigenpy support, i.e. numpy/eigen compatibility.
    eigenpy::enableEigenPy();
//     aig::python::exposeContact6D();

// boost::python::class_<aig::Contact6D>("Contact6D", boost::python::init<>());

    aig::python::exposeContact6DSettings();
//     boost::python::class_<aig::Contact6DSettings>("Contact6DSettings");
}
