#include <boost/test/unit_test.hpp>

#include "aig/unittests/pyrene_settings.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_leg_ig_constructor) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(aig::unittests::urdf,
                              pinocchio::JointModelFreeFlyer(), model);
  BOOST_CHECK_EQUAL(model.name, "talos");
}

BOOST_AUTO_TEST_SUITE_END()
