#include "aig/dyna_com.hpp"

#include <pinocchio/multibody/fwd.hpp>
// Must be included first!
#include <boost/python.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>

#include "aig/contact6d.hpp"
#include "aig/python.hpp"

namespace aig {
namespace python {
namespace bp = boost::python;

bp::dict get_settings(DynaCoM &self) {
  bp::dict settings;
  DynaCoMSettings conf = self.getSettings();
  settings["urdf"] = conf.urdf;

  return settings;
}

std::shared_ptr<Contact6D> getContact(DynaCoM &self, std::string name) {
  return boost::ref(self.getContact(name));
}

void exposeDynaCoM() {
  bp::class_<std::vector<std::string>>("names_vector")
      .def(bp::vector_indexing_suite<std::vector<std::string>>());

  bp::class_<DynaCoMSettings>("DynaCoMSettings")
      .def_readwrite("urdf", &DynaCoMSettings::urdf);

  bp::class_<DynaCoM, boost::noncopyable>("DynaCoM", bp::init<>())
      .def("initialize", &DynaCoM::initialize, bp::args("self", "settings"))
      .def("computeDynamics", &DynaCoM::computeDynamics,
           (bp::arg("self"), bp::arg("posture"), bp::arg("velocity"),
            bp::arg("aceleration"), bp::arg("extWrench"),
            bp::arg("flatHorizontalGround") = true))
      .def<void (DynaCoM::*)(const double &, const Eigen::VectorXd &,
                             const Eigen::VectorXd &, const Eigen::VectorXd &,
                             const Eigen::Matrix<double, 6, 1> &,
                             bool flatHorizontalGround)>(
          "computeNL", &DynaCoM::computeNL,
          bp::args("self", "posture", "velocity", "aceleration", "extWrench",
                   "flatHorizontalGround"))
      .def<void (DynaCoM::*)(const double &)>("computeNL", &DynaCoM::computeNL,
                                              bp::args("self"))
      .def("addContact6d", &DynaCoM::addContact6d,
           (bp::arg("self"), bp::arg("contact"), bp::arg("name"),
            bp::arg("active") = true))
      .def("removeContact6d", &DynaCoM::removeContact6d,
           bp::args("self", "name"))
      .def("activateContact6d", &DynaCoM::activateContact6d,
           bp::args("self", "name"))
      .def("deactivateContact6d", &DynaCoM::deactivateContact6d,
           bp::args("self", "name"))
      .def("distributeForce", &DynaCoM::distributeForce,
           bp::args("self", "groundCoMForce", "groundCoMTorque", "CoM"))
      .def("getContact", &getContact, bp::args("self", "name"))
      .def("getActiveContacts", &DynaCoM::getActiveContacts,
           bp::return_value_policy<bp::copy_const_reference>(),
           bp::args("self"))
      .def("uni_A", &DynaCoM::uni_A, bp::args("self"))
      .def("uni_b", &DynaCoM::uni_b, bp::args("self"))
      .def("fri_A", &DynaCoM::fri_A, bp::args("self"))
      .def("fri_b", &DynaCoM::fri_b, bp::args("self"))
      .def("reg_A", &DynaCoM::reg_A, bp::args("self"))
      .def("reg_b", &DynaCoM::reg_b, bp::args("self"))
      .def("NE_A", &DynaCoM::NE_A, bp::args("self"))
      .def("NE_b", &DynaCoM::NE_b,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("allForces", &DynaCoM::allForces,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("model", &DynaCoM::getModel,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("data", &DynaCoM::getData,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("getCoM", &DynaCoM::getCoM,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("getVCoM", &DynaCoM::getVCoM,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("getACoM", &DynaCoM::getACoM,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("getAM", &DynaCoM::getAM,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("getCoP", &DynaCoM::getCoP,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("getNL", &DynaCoM::getNL,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("getGroundCoMForce", &DynaCoM::getGroundCoMForce,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"))
      .def("getGroundCoMTorque", &DynaCoM::getGroundCoMTorque,
           bp::return_value_policy<bp::reference_existing_object>(),
           bp::args("self"));
}
} // namespace python
} // namespace aig
