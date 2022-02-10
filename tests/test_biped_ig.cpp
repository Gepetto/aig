#include <boost/test/unit_test.hpp>

#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "aig/biped_ig.hpp"
#include "aig/unittests/pyrene_settings.hpp"

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_biped_ig_default_constructor) {
  aig::BipedIG biped_ig;
  aig::BipedIGSettings default_settings;
  BOOST_CHECK_EQUAL(biped_ig.get_settings(), default_settings);
}

BOOST_AUTO_TEST_CASE(test_biped_ig_init_constructor) {
  aig::BipedIGSettings settings = aig::unittests::bipeds;

  aig::BipedIG biped_ig(settings);
  BOOST_CHECK_EQUAL(biped_ig.get_settings(), settings);
}

BOOST_AUTO_TEST_CASE(test_leg_ig_init_through_biped_ig) {
  aig::BipedIGSettings settings = aig::unittests::bipeds;

  aig::BipedIG biped_ig(settings);

  const aig::LegIGSettings& llegs = biped_ig.get_left_leg_settings();
  const aig::LegIGSettings& rlegs = biped_ig.get_right_leg_settings();

  aig::LegIGSettings test_llegs = aig::unittests::llegs;
  aig::LegIGSettings test_rlegs = aig::unittests::rlegs;

  BOOST_CHECK_EQUAL(llegs, test_llegs);
  BOOST_CHECK_EQUAL(rlegs, test_rlegs);
}

enum Mode { ZERO, HALF_SITTING, RANDOM };

void generate_references(Eigen::Vector3d& com, pinocchio::SE3& base,
                         pinocchio::SE3& lf, pinocchio::SE3& rf,
                         Eigen::VectorXd& q, const Mode& mode) {
  // Get the model and data
  pinocchio::Model model;
  pinocchio::urdf::buildModel(aig::unittests::urdf_path,
                              pinocchio::JointModelFreeFlyer(), model);
  pinocchio::Data data = pinocchio::Data(model);
  pinocchio::srdf::loadReferenceConfigurations(
      model, aig::unittests::srdf_path, false);

  // Generate a robot configuration.
  switch (mode) {
    case Mode::ZERO:
      q = Eigen::VectorXd::Zero(model.nq);
      q(6) = 1.0;
      break;
    case Mode::HALF_SITTING:
      q = model.referenceConfigurations["half_sitting"];
      break;
    case Mode::RANDOM:
      model.lowerPositionLimit.head<3>().fill(-0.1);
      model.upperPositionLimit.head<3>().fill(0.1);
      q = randomConfiguration(model);
      break;
    default:
      throw std::runtime_error(
          "tes_leg_ig: generate_references():Switch default ask, not "
          "implemented.");
      break;
  }

  // Forward Kinematics.
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::FrameIndex lf_id =
      model.getFrameId(aig::unittests::left_foot_frame_name);
  pinocchio::FrameIndex rf_id =
      model.getFrameId(aig::unittests::right_foot_frame_name);

  // outputs
  lf = data.oMf[lf_id];
  rf = data.oMf[rf_id];
  Eigen::Quaterniond quat = Eigen::Quaterniond(q[6], q[3], q[4], q[5]);
  quat.normalize();
  base = pinocchio::SE3(quat.toRotationMatrix(), q.head<3>());
  com = pinocchio::centerOfMass(model, data, q);
}

void test_solve(Mode mode) {
  // create the solver
  aig::BipedIGSettings settings = aig::unittests::bipeds;
  aig::BipedIG biped_ig(settings);

  // perform a forward kinematics on a configuration
  Eigen::VectorXd q_test, q_ig_com, q_ig_com_baserot, q_ig_base;
  Eigen::Vector3d com;
  pinocchio::SE3 base, lf, rf;
  generate_references(com, base, lf, rf, q_test, mode);
  double precision = mode == Mode::RANDOM ? 1.0 : 1e-3;

  // Compute inverse geometry and tests
  biped_ig.solve(com, lf, rf, q_test, q_ig_com);
  BOOST_CHECK_EQUAL(q_test.size(), q_ig_com.size());
  // BOOST_CHECK_LE((q_test - q_ig_com).norm(), precision);
  //
  biped_ig.solve(com, base.rotation(), lf, rf, q_test, q_ig_com_baserot);
  BOOST_CHECK_EQUAL(q_test.size(), q_ig_com_baserot.size());
  // BOOST_CHECK_LE((q_test - q_ig_com_baserot).norm(), precision);
  //
  biped_ig.solve(base, lf, rf, q_test, q_ig_base);
  BOOST_CHECK_EQUAL(q_test.size(), q_ig_base.size());
  BOOST_CHECK_LE((q_test - q_ig_base).norm(), precision);
}

BOOST_AUTO_TEST_CASE(test_solve_zero) { test_solve(Mode::ZERO); }

BOOST_AUTO_TEST_CASE(test_solve_half_sitting) {
  test_solve(Mode::HALF_SITTING);
}

BOOST_AUTO_TEST_CASE(test_solve_random) { test_solve(Mode::RANDOM); }

BOOST_AUTO_TEST_SUITE_END()
