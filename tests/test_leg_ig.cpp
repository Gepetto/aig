#include <boost/test/unit_test.hpp>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "preview_ik/unittests/pyrene_settings.hpp"
#include "preview_ik/leg_ig.hpp"

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_leg_ig_default_constructor) {
  preview_ik::LegIG leg_ig;
  preview_ik::LegIGSettings default_settings;
  BOOST_CHECK_EQUAL(leg_ig.get_settings(), default_settings);
}

BOOST_AUTO_TEST_CASE(test_leg_ig_init_constructor) {
  preview_ik::LegIGSettings settings;

  settings.side = preview_ik::LegIGSettings::Side::LEFT;
  settings.femur_length = 0.05;
  settings.tibia_length = 0.2;
  settings.hip_from_waist = Eigen::Vector3d::Random();
  settings.ankle_from_foot = Eigen::Vector3d::Random();

  preview_ik::LegIG leg_ig(settings);
  BOOST_CHECK_EQUAL(leg_ig.get_settings(), settings);
}

void generate_references(pinocchio::SE3& base, pinocchio::SE3& lf, pinocchio::SE3& rf, Eigen::VectorXd& q, const bool& random) {
  // Get the model and data
  pinocchio::Model model;
  pinocchio::urdf::buildModel(preview_ik::unittests::urdf_path, pinocchio::JointModelFreeFlyer(), model);
  pinocchio::Data data = pinocchio::Data(model);
  pinocchio::srdf::loadReferenceConfigurations(model, preview_ik::unittests::srdf_path, false);

  // Generate a robot configuration.
  if (random) {
    q = randomConfiguration(model);
  } else {
    q = model.referenceConfigurations["half_sitting"];
  }

  // Forward Kinematics.
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::FrameIndex lf_id = model.getFrameId(preview_ik::unittests::left_foot_frame_name);
  pinocchio::FrameIndex rf_id = model.getFrameId(preview_ik::unittests::right_foot_frame_name);

  lf = data.oMf[lf_id];
  rf = data.oMf[rf_id];
  Eigen::Quaterniond quat = Eigen::Quaterniond(q[6], q[3], q[4], q[5]);
  quat.normalize();
  base = pinocchio::SE3(quat.toRotationMatrix(), q.head<3>());
}

BOOST_AUTO_TEST_CASE(test_leg_ig_solve_left_half_sitting) {
  // create the solver
  preview_ik::LegIGSettings settings;
  settings.side = preview_ik::LegIGSettings::Side::LEFT;
  settings.femur_length = 0.38;
  settings.tibia_length = 0.325;
  settings.hip_from_waist << -0.02, 0.085, -0.27105;
  settings.ankle_from_foot << -0, -0, 0.107;
  preview_ik::LegIG leg_ig(settings);

  // perform a forward kinematics on a configuration
  Eigen::VectorXd q;
  pinocchio::SE3 base, lf, rf; 
  generate_references(base, lf, rf, q, false);

  // Compute inverse geometry.
  preview_ik::LegJoints leg_joints = leg_ig.solve(base, lf);
  
  
  // Tests
  BOOST_CHECK_EQUAL(leg_ig.get_settings(), settings);
}

BOOST_AUTO_TEST_SUITE_END()
