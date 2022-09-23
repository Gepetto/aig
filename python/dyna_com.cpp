#include <pinocchio/multibody/fwd.hpp>  // Must be included first!
#include <boost/python.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>

#include "aig/dyna_com.hpp"
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
  bp::class_<std::vector<std::string> >("names_vector")
    .def(bp::vector_indexing_suite<std::vector<std::string> >())
  ;

  bp::class_<DynaCoMSettings>("DynaCoMSettings")
    .def_readwrite("urdf", &DynaCoMSettings::urdf)
  ;

  bp::class_<DynaCoM, boost::noncopyable>("DynaCoM", bp::init<>())
    .def("initialize", &DynaCoM::initialize, bp::args("self", "settings"))
    .def("computeDynamics", &DynaCoM::computeDynamics, bp::args("self", "posture", "velocity", "aceleration", "extWrench", "flatHorizontalGround"))
    .def<void (DynaCoM::*)(const double &, const Eigen::VectorXd &, 
                                           const Eigen::VectorXd &,
                                           const Eigen::VectorXd &,
                                           const Eigen::Matrix<double, 6, 1> &,
                                           bool flatHorizontalGround)>("computeNL", &DynaCoM::computeNL, bp::args("self", "posture", "velocity", "aceleration", "extWrench", "flatHorizontalGround"))
    .def<void (DynaCoM::*)(const double &)>("computeNL", &DynaCoM::computeNL, bp::args("self"))
    .def("addContact6d", &DynaCoM::addContact6d, (bp::arg("self"), bp::arg("contact"), bp::arg("name"), bp::arg("active")=true))
    .def("removeContact6d", &DynaCoM::removeContact6d, bp::args("self", "name"))
    .def("activateContact6d", &DynaCoM::activateContact6d, bp::args("self", "name"))
    .def("deactivateContact6d", &DynaCoM::deactivateContact6d, bp::args("self", "name"))
    .def("distributeForce", &DynaCoM::distributeForce, bp::args("self", "groundCoMForce", "groundCoMTorque", "CoM"))
    .def("getContact", &getContact, bp::args("self", "name"))
    .def("getActiveContacts", &DynaCoM::getActiveContacts, bp::return_value_policy<bp::copy_const_reference>(), bp::args("self"))
  ;
}
}  // namespace python
}  // namespace aig