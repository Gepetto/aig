#include <boost/test/unit_test.hpp>

#include "aig/biped_ig.hpp"
#include "aig/unittests/pyrene_settings.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_biped_ig_talos_settings) {
  const std::string path_to_robots = EXAMPLE_ROBOT_DATA_MODEL_DIR "/talos_data";
  aig::BipedIGSettings talos_settings =
      aig::makeSettingsFor(path_to_robots, "talos");
  aig::BipedIG biped_ig(talos_settings);
  BOOST_CHECK_EQUAL(biped_ig.get_settings(), talos_settings);
}

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

BOOST_AUTO_TEST_CASE(test_biped_ig_init_constructor_urdf_content) {
  aig::BipedIGSettings settings = aig::unittests::bipeds;
  aig::BipedIG biped_ig_1(settings);

  // read the urdf:
  std::ifstream file(settings.urdf.c_str());
  std::stringstream buffer;
  buffer << file.rdbuf();
  // Save the urdf content into a string
  settings.urdf = buffer.str();
  aig::BipedIG biped_ig_2(settings);

  // Check that both pinocchio model are equal.
  BOOST_CHECK_EQUAL(biped_ig_1.get_model(), biped_ig_2.get_model());
}

BOOST_AUTO_TEST_CASE(test_biped_ig_init_constructor_srdf_content) {
  aig::BipedIGSettings settings = aig::unittests::bipeds;
  aig::BipedIG biped_ig_1(settings);

  // read the urdf:
  std::ifstream file(settings.srdf.c_str());
  std::stringstream buffer;
  buffer << file.rdbuf();
  // Save the urdf content into a string
  settings.srdf = buffer.str();
  aig::BipedIG biped_ig_2(settings);

  // Check that both pinocchio model are equal.
  BOOST_CHECK_EQUAL(biped_ig_1.get_model(), biped_ig_2.get_model());
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
  pinocchio::urdf::buildModel(aig::unittests::urdf,
                              pinocchio::JointModelFreeFlyer(), model);
  pinocchio::Data data = pinocchio::Data(model);
  pinocchio::srdf::loadReferenceConfigurations(model, aig::unittests::srdf,
                                               false);

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
  double precision = mode == Mode::RANDOM ? 1.0 : 1e-4;

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

void test_solve_derivatives(Mode mode) {
  // create the solver
  aig::BipedIGSettings settings = aig::unittests::bipeds;
  aig::BipedIG biped_ig(settings);

  // perform a forward kinematics on a configuration
  Eigen::VectorXd q_test, q_ig_com, q_ig_com_baserot, q_ig_base, v_ig_com,
      a_ig_com;
  Eigen::Vector3d com;
  pinocchio::SE3 base, lf, rf;  //
  generate_references(com, base, lf, rf, q_test, mode);
  // double precision = mode == Mode::RANDOM ? 1.0 : 1e-3;

  double dt = 1e-5;
  // std::array<pinocchio::SE3, 3> bases{ {base, base, base} };
  std::array<Eigen::Vector3d, 3> coms{{com, com, com}};
  std::array<pinocchio::SE3, 3> lfs{{lf, lf, lf}};
  std::array<pinocchio::SE3, 3> rfs{{rf, rf, rf}};

  // Compute inverse geometry and tests
  biped_ig.solve(coms, lfs, rfs, q_test, q_ig_com, v_ig_com, a_ig_com, dt);
  BOOST_CHECK_EQUAL(q_test.size(), q_ig_com.size());
  BOOST_CHECK_EQUAL(q_test.size() - 1, v_ig_com.size());
  BOOST_CHECK_EQUAL(q_test.size() - 1, a_ig_com.size());

  // Compute with Eigen::Isometry3d instead of pinocchio::SE3
  Eigen::Isometry3d LF, RF;  //()//(lf.toHomogeneousMatrix())
  LF.rotate(lf.rotation());
  LF.translate(lf.translation());
  RF.rotate(rf.rotation());
  RF.translate(rf.translation());
  std::array<Eigen::Isometry3d, 3> LFs{{LF, LF, LF}};
  std::array<Eigen::Isometry3d, 3> RFs{{RF, RF, RF}};

  biped_ig.solve(coms, LFs, RFs, q_test, q_ig_com, v_ig_com, a_ig_com, dt);
  BOOST_CHECK_EQUAL(q_test.size(), q_ig_com.size());
  BOOST_CHECK_EQUAL(q_test.size() - 1, v_ig_com.size());
  BOOST_CHECK_EQUAL(q_test.size() - 1, a_ig_com.size());

  /*BOOST_CHECK_LE((q_test - q_ig_com).norm(), precision);

  biped_ig.solve(com, base.rotation(), lf, rf, q_test, q_ig_com_baserot);
  BOOST_CHECK_EQUAL(q_test.size(), q_ig_com_baserot.size());
  // BOOST_CHECK_LE((q_test - q_ig_com_baserot).norm(), precision);
  //
  biped_ig.solve(base, lf, rf, q_test, q_ig_base);
  BOOST_CHECK_EQUAL(q_test.size(), q_ig_base.size());
  BOOST_CHECK_LE((q_test - q_ig_base).norm(), precision);*/
}

BOOST_AUTO_TEST_CASE(test_solve_half_sitting_derivatives) {
  test_solve_derivatives(Mode::HALF_SITTING);
}

BOOST_AUTO_TEST_CASE(test_compute_dynamics) {
  // create the solver
  aig::BipedIGSettings settings = aig::unittests::bipeds;
  aig::BipedIG biped_ig(settings);

  // perform a forward kinematics on a configuration
  Eigen::VectorXd q_test, q_ig_com, v_ig_com, a_ig_com;
  Eigen::Vector3d com;
  pinocchio::SE3 base, lf, rf;  //
  generate_references(com, base, lf, rf, q_test, Mode::HALF_SITTING);

  double dt = 1e-5;
  // std::array<pinocchio::SE3, 3> bases{ {base, base, base} };
  std::array<Eigen::Vector3d, 3> coms{{com, com, com}};
  std::array<pinocchio::SE3, 3> lfs{{lf, lf, lf}};
  std::array<pinocchio::SE3, 3> rfs{{rf, rf, rf}};

  // Compute inverse geometry and tests
  biped_ig.solve(coms, lfs, rfs, q_test, q_ig_com, v_ig_com, a_ig_com, dt);
  BOOST_CHECK_EQUAL(q_test.size(), q_ig_com.size());
  BOOST_CHECK_EQUAL(q_test.size() - 1, v_ig_com.size());
  BOOST_CHECK_EQUAL(q_test.size() - 1, a_ig_com.size());
  BOOST_CHECK(v_ig_com.isMuchSmallerThan(1));
  BOOST_CHECK(a_ig_com.isMuchSmallerThan(1));

  biped_ig.computeNL(3.3, q_ig_com, v_ig_com, a_ig_com);

  BOOST_CHECK(biped_ig.getAM().isMuchSmallerThan(1));
  BOOST_CHECK(biped_ig.getAMVariation().isMuchSmallerThan(1));
  BOOST_CHECK(biped_ig.getNL().isMuchSmallerThan(1));
  // BOOST_CHECK(
  //     (biped_ig.getCoM().head<2>() -
  //     biped_ig.getCoP()).isMuchSmallerThan(1));
}

BOOST_AUTO_TEST_SUITE_END()
