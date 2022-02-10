#include <boost/test/tools/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>

#include "aig/leg_ig.hpp"
#include "aig/unittests/pyrene_settings.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_default_constructor) {
  aig::LegIG leg_ig;
  aig::LegIGSettings default_settings;
  BOOST_CHECK_EQUAL(leg_ig.get_settings(), default_settings);
}

BOOST_AUTO_TEST_CASE(test_init_constructor) {
  aig::LegIGSettings settings;

  // Randomize the Matrices.
  srand((unsigned int)time(0));
  settings.knee_from_hip = Eigen::Vector3d::Random();
  settings.ankle_from_knee = Eigen::Vector3d::Random();
  settings.hip_from_waist = Eigen::Vector3d::Random();
  settings.ankle_from_foot = Eigen::Vector3d::Random();

  aig::LegIG leg_ig(settings);
  BOOST_CHECK_EQUAL(leg_ig.get_settings(), settings);
}

enum Mode { ZERO, HALF_SITTING, RANDOM };

void generate_references(pinocchio::SE3& base, pinocchio::SE3& lf,
                         pinocchio::SE3& rf, aig::LegJoints& ll_q,
                         aig::LegJoints& rl_q, const Mode& mode) {
  // Get the model and data
  pinocchio::Model model;
  pinocchio::urdf::buildModel(aig::unittests::urdf_path,
                              pinocchio::JointModelFreeFlyer(), model);
  pinocchio::Data data = pinocchio::Data(model);
  pinocchio::srdf::loadReferenceConfigurations(model, aig::unittests::srdf_path,
                                               false);

  // Generate a robot configuration.
  Eigen::VectorXd q;
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

  // Get the legs joints configuration for the test
  int lleg_idx_qs =
      model.idx_qs[model.getJointId(aig::unittests::left_hip_joint_name)];
  int rleg_idx_qs =
      model.idx_qs[model.getJointId(aig::unittests::right_hip_joint_name)];

  // outputs
  lf = data.oMf[lf_id];
  rf = data.oMf[rf_id];
  Eigen::Quaterniond quat = Eigen::Quaterniond(q[6], q[3], q[4], q[5]);
  quat.normalize();
  base = pinocchio::SE3(quat.toRotationMatrix(), q.head<3>());
  ll_q = q.segment<6>(lleg_idx_qs);
  rl_q = q.segment<6>(rleg_idx_qs);
}

void test_solve(bool left, Mode mode) {
  // create the solver
  aig::LegIGSettings settings;
  if (left) {
    settings = aig::unittests::llegs;
  } else {
    settings = aig::unittests::rlegs;
  }
  aig::LegIG leg_ig(settings);

  // perform a forward kinematics on a configuration
  Eigen::VectorXd q;
  aig::LegJoints test_ll_q, test_rl_q;
  pinocchio::SE3 base, lf, rf;
  generate_references(base, lf, rf, test_ll_q, test_rl_q, mode);
  double precision = mode == Mode::RANDOM ? 1.0 : 1e-3;
  // Compute inverse geometry.
  aig::LegJoints q_leg;
  if (left) {
    q_leg = leg_ig.solve(base, lf);
  } else {
    q_leg = leg_ig.solve(base, rf);
  }

  // Tests.
  if (left) {
    BOOST_CHECK_LE((q_leg - test_ll_q).norm(), precision);
  } else {
    BOOST_CHECK_LE((q_leg - test_rl_q).norm(), precision);
  }
}

BOOST_AUTO_TEST_CASE(test_solve_left_zero) { test_solve(true, Mode::ZERO); }

BOOST_AUTO_TEST_CASE(test_solve_left_half_sitting) {
  test_solve(true, Mode::HALF_SITTING);
}

BOOST_AUTO_TEST_CASE(test_solve_left_random) {
  // Randomize the Matrices.
  srand((unsigned int)time(0));
  test_solve(true, Mode::RANDOM);
}

BOOST_AUTO_TEST_CASE(test_solve_right_zero) { test_solve(false, Mode::ZERO); }

BOOST_AUTO_TEST_CASE(test_solve_right_half_sitting) {
  test_solve(false, Mode::HALF_SITTING);
}

BOOST_AUTO_TEST_CASE(test_solve_right_random) {
  // Randomize the Matrices.
  srand((unsigned int)time(0));
  test_solve(false, Mode::RANDOM);
}

BOOST_AUTO_TEST_SUITE_END()
