#include <boost/test/unit_test.hpp>

#include "preview_ik/biped_ig.hpp"
#include "preview_ik/unittests/pyrene_settings.hpp"

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_biped_ig_default_constructor) {
  preview_ik::BipedIG biped_ig;
  preview_ik::BipedIGSettings default_settings;
  BOOST_CHECK_EQUAL(biped_ig.get_settings(), default_settings);
}

BOOST_AUTO_TEST_CASE(test_biped_ig_init_constructor) {
  preview_ik::BipedIGSettings settings;

  settings.left_hip_joint_name = preview_ik::unittests::left_hip_joint_name;
  settings.left_knee_joint_name = preview_ik::unittests::left_knee_joint_name;
  settings.left_ankle_joint_name = preview_ik::unittests::left_ankle_joint_name;
  settings.left_foot_frame_name = preview_ik::unittests::left_foot_frame_name;
  settings.right_hip_joint_name = preview_ik::unittests::right_hip_joint_name;
  settings.right_knee_joint_name = preview_ik::unittests::right_knee_joint_name;
  settings.right_ankle_joint_name =
      preview_ik::unittests::right_ankle_joint_name;
  settings.right_foot_frame_name = preview_ik::unittests::right_foot_frame_name;
  settings.urdf_path = preview_ik::unittests::urdf_path;
  settings.srdf_path = preview_ik::unittests::srdf_path;

  preview_ik::BipedIG biped_ig(settings);
  BOOST_CHECK_EQUAL(biped_ig.get_settings(), settings);
}

BOOST_AUTO_TEST_CASE(test_leg_ig_init_through_biped_ig) {
  preview_ik::BipedIGSettings settings;

  settings.left_hip_joint_name = preview_ik::unittests::left_hip_joint_name;
  settings.left_knee_joint_name = preview_ik::unittests::left_knee_joint_name;
  settings.left_ankle_joint_name = preview_ik::unittests::left_ankle_joint_name;
  settings.left_foot_frame_name = preview_ik::unittests::left_foot_frame_name;
  settings.right_hip_joint_name = preview_ik::unittests::right_hip_joint_name;
  settings.right_knee_joint_name = preview_ik::unittests::right_knee_joint_name;
  settings.right_ankle_joint_name =
      preview_ik::unittests::right_ankle_joint_name;
  settings.right_foot_frame_name = preview_ik::unittests::right_foot_frame_name;
  settings.urdf_path = preview_ik::unittests::urdf_path;
  settings.srdf_path = preview_ik::unittests::srdf_path;

  preview_ik::BipedIG biped_ig(settings);

  const preview_ik::LegIGSettings& llegs = biped_ig.get_left_leg_settings();
  const preview_ik::LegIGSettings& rlegs = biped_ig.get_right_leg_settings();

  preview_ik::LegIGSettings test_llegs = preview_ik::unittests::llegs;
  preview_ik::LegIGSettings test_rlegs = preview_ik::unittests::rlegs;

  BOOST_CHECK_EQUAL(llegs, test_llegs);
  BOOST_CHECK_EQUAL(rlegs, test_rlegs);
}

BOOST_AUTO_TEST_SUITE_END()
