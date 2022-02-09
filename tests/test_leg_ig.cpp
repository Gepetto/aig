#include <boost/test/tools/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "preview_ik/leg_ig.hpp"
#include "preview_ik/unittests/pyrene_settings.hpp"

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_leg_ig_default_constructor) {
  preview_ik::LegIG leg_ig;
  preview_ik::LegIGSettings default_settings;
  BOOST_CHECK_EQUAL(leg_ig.get_settings(), default_settings);
}

BOOST_AUTO_TEST_CASE(test_leg_ig_init_constructor) {
  preview_ik::LegIGSettings settings;

  // Randomize the Matrices.
  srand((unsigned int) time(0));
  settings.knee_from_hip = Eigen::Vector3d::Random();
  settings.ankle_from_knee = Eigen::Vector3d::Random();
  settings.hip_from_waist = Eigen::Vector3d::Random();
  settings.ankle_from_foot = Eigen::Vector3d::Random();

  preview_ik::LegIG leg_ig(settings);
  BOOST_CHECK_EQUAL(leg_ig.get_settings(), settings);
}

enum Mode { ZERO, HALF_SITTING, RANDOM };

void generate_references(pinocchio::SE3& base, pinocchio::SE3& lf,
                         pinocchio::SE3& rf, preview_ik::LegJoints& ll_q,
                         preview_ik::LegJoints& rl_q, const Mode& mode) {
  // Get the model and data
  pinocchio::Model model;
  pinocchio::urdf::buildModel(preview_ik::unittests::urdf_path,
                              pinocchio::JointModelFreeFlyer(), model);
  pinocchio::Data data = pinocchio::Data(model);
  pinocchio::srdf::loadReferenceConfigurations(
      model, preview_ik::unittests::srdf_path, false);

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
      model.getFrameId(preview_ik::unittests::left_foot_frame_name);
  pinocchio::FrameIndex rf_id =
      model.getFrameId(preview_ik::unittests::right_foot_frame_name);

  // Get the legs joints configuration for the test
  int lleg_idx_qs =
      model
          .idx_qs[model.getJointId(preview_ik::unittests::left_hip_joint_name)];
  int rleg_idx_qs = model.idx_qs[model.getJointId(
      preview_ik::unittests::right_hip_joint_name)];

  // outputs
  lf = data.oMf[lf_id];
  rf = data.oMf[rf_id];
  Eigen::Quaterniond quat = Eigen::Quaterniond(q[6], q[3], q[4], q[5]);
  quat.normalize();
  base = pinocchio::SE3(quat.toRotationMatrix(), q.head<3>());
  ll_q = q.segment<6>(lleg_idx_qs);
  rl_q = q.segment<6>(rleg_idx_qs);
}

BOOST_AUTO_TEST_CASE(test_leg_ig_solve_left_zero) {
  // create the solver
  preview_ik::LegIGSettings settings = preview_ik::unittests::llegs;
  preview_ik::LegIG leg_ig(settings);

  // perform a forward kinematics on a configuration
  Eigen::VectorXd q;
  preview_ik::LegJoints test_ll_q, test_rl_q;
  pinocchio::SE3 base, lf, rf;
  generate_references(base, lf, rf, test_ll_q, test_rl_q, Mode::ZERO);

  // Compute inverse geometry.
  preview_ik::LegJoints ll_q = leg_ig.solve(base, lf);

  // Tests.
  BOOST_CHECK((ll_q - test_ll_q).norm() < 1e-3);
}

BOOST_AUTO_TEST_CASE(test_leg_ig_solve_left_half_sitting) {
  // create the solver
  preview_ik::LegIGSettings settings = preview_ik::unittests::llegs;
  preview_ik::LegIG leg_ig(settings);

  // perform a forward kinematics on a configuration
  Eigen::VectorXd q;
  preview_ik::LegJoints test_ll_q, test_rl_q;
  pinocchio::SE3 base, lf, rf;
  generate_references(base, lf, rf, test_ll_q, test_rl_q, Mode::HALF_SITTING);

  // Compute inverse geometry.
  preview_ik::LegJoints ll_q = leg_ig.solve(base, lf);

  // Tests.
  BOOST_CHECK((ll_q - test_ll_q).norm() < 1e-3);
}

BOOST_AUTO_TEST_CASE(test_leg_ig_solve_left_random) {
  // Randomize the Matrices.
  srand((unsigned int) time(0));

  // create the solver
  preview_ik::LegIGSettings settings = preview_ik::unittests::llegs;
  preview_ik::LegIG leg_ig(settings);

  // perform a forward kinematics on a configuration
  Eigen::VectorXd q;
  preview_ik::LegJoints test_ll_q, test_rl_q;
  pinocchio::SE3 base, lf, rf;
  generate_references(base, lf, rf, test_ll_q, test_rl_q, Mode::RANDOM);

  // Compute inverse geometry.
  preview_ik::LegJoints ll_q = leg_ig.solve(base, lf);

  // Tests.
  std::cout << "test_ll_q = " << test_ll_q.transpose() << std::endl;
  std::cout << "ll_q = " << ll_q.transpose() << std::endl;
  std::cout << "ll_q - test_ll_q = " << (ll_q - test_ll_q).norm() << std::endl;
  BOOST_CHECK((ll_q - test_ll_q).norm() < 0.5);
}

BOOST_AUTO_TEST_SUITE_END()
