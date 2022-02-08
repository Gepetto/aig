#include <boost/test/unit_test.hpp>

#include "example-robot-data/path.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "preview_ik/unittests/pyrene_settings.hpp"

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_load_talos_model) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(preview_ik::unittests::urdf_path,
                              pinocchio::JointModelFreeFlyer(), model);
  BOOST_CHECK_EQUAL(model.name, "talos");
}

BOOST_AUTO_TEST_CASE(test_get_reference_config) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(preview_ik::unittests::urdf_path,
                              pinocchio::JointModelFreeFlyer(), model);
  pinocchio::srdf::loadReferenceConfigurations(
      model, preview_ik::unittests::srdf_path, false);
  Eigen::VectorXd q = model.referenceConfigurations["half_sitting"];
  BOOST_CHECK_EQUAL(q.size(), model.nq);
}

BOOST_AUTO_TEST_CASE(test_compute_joint_placement) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(preview_ik::unittests::urdf_path,
                              pinocchio::JointModelFreeFlyer(), model);
  pinocchio::srdf::loadReferenceConfigurations(
      model, preview_ik::unittests::srdf_path, false);
  Eigen::VectorXd q = model.referenceConfigurations["half_sitting"];

  Eigen::Vector3d test;
  test << -0.02, 0.085, -0.27105;
  BOOST_CHECK_EQUAL(test, model.jointPlacements[2].translation());
}

BOOST_AUTO_TEST_CASE(test_compute_com) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(preview_ik::unittests::urdf_path,
                              pinocchio::JointModelFreeFlyer(), model);
  pinocchio::srdf::loadReferenceConfigurations(
      model, preview_ik::unittests::srdf_path, false);
  Eigen::VectorXd q = model.referenceConfigurations["half_sitting"];
  pinocchio::Data data(model);
  Eigen::Vector3d com = pinocchio::centerOfMass(model, data, q);

  Eigen::Vector3d test;
  test << -0.0031639, 0.00123738, 0.876681;
  BOOST_CHECK(test.isApprox(com, 1e-5));
}

BOOST_AUTO_TEST_CASE(test_eigen_norms) {
  Eigen::Vector3d x(3, 4, 0);
  BOOST_CHECK_EQUAL(x.norm(), 5.0);
}

BOOST_AUTO_TEST_CASE(test_se3) {
  pinocchio::SE3 se3;
  se3.setIdentity();
  BOOST_CHECK_EQUAL(se3.rotation(), Eigen::Matrix3d::Identity());
  BOOST_CHECK_EQUAL(se3.translation(), Eigen::Vector3d::Zero());
}

BOOST_AUTO_TEST_CASE(test_skew) {
  Eigen::Vector3d v;
  v << 2, 3, 4;
  Eigen::Matrix3d skew_mat;
  skew_mat << 0, -4, 3, 4, 0, -2, -3, 2, 0;
  BOOST_CHECK_EQUAL(pinocchio::skew(v), skew_mat);
}

BOOST_AUTO_TEST_SUITE_END()
