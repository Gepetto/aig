#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct  7 15:13:50 2022

@author: nvilla
"""
import aig
import numpy as np
from example_robot_data.path import EXAMPLE_ROBOT_DATA_MODEL_DIR


import unittest
import pinocchio as pin
from time import sleep, time

debug = False
visualize = False

# unittest.util._MAX_LENGTH = 2000


def get_visualizer():
    import os
    import subprocess
    from example_robot_data.robots_loader import load

    robot = load("talos")

    # Launching geppeto gui
    launched = subprocess.getstatusoutput(
        "ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l"
    )
    if int(launched[1]) == 0:
        os.system("gepetto-gui &")

    viewer = pin.visualize.GepettoVisualizer
    visual = viewer(robot.model, robot.collision_model, robot.visual_model)
    visual.initViewer(loadModel=True)
    visual.displayCollisions(False)
    visual.displayVisuals(True)
    visual.display(robot.q0)

    gui = visual.viewer.gui
    gui.setLightingMode("world/ground", "OFF")

    return visual  # then use:   visual.display(q)


class TestBipedIG(unittest.TestCase):
    def setUp(self):

        settings = aig.BipedIGSettings.makeSettingsFor(
            EXAMPLE_ROBOT_DATA_MODEL_DIR + "/talos_data", "talos"
        )
        biped = aig.BipedIG()
        biped.initialize(settings)

        model = biped.model()

        leftFrameID = model.getFrameId("leg_left_sole_fix_joint")
        rightFrameID = model.getFrameId("leg_right_sole_fix_joint")

        self.biped = biped
        self.q0 = biped.getQ0()
        self.model = model
        self.data = biped.data()
        self.LF_id = leftFrameID
        self.RF_id = rightFrameID
        self.Base_id = 1

    def test_base_postures(self):
        e = 1e-11
        if visualize:
            v = get_visualizer()

        L_pos = [
            np.array([0.0, 0.28, 0]),
            np.array([0.04, 0.05, 0]),
            np.array([-0.05, 0, 0]),
            np.array([0.1, 0.4, 0.3]),
        ]
        R_pos = [
            np.array([0.0, -0.38, 0]),
            np.array([-0.14, -0.15, 0]),
            np.array([-0.52, 0, 0]),
            np.array([-0.3, 0, 0.15]),
        ]
        BS = pin.SE3(np.eye(3), np.array([0, 0, 0.8]))

        for L, R in zip(L_pos, R_pos):
            LF = pin.SE3(np.eye(3), L)
            RF = pin.SE3(np.eye(3), R)

            posture = self.biped.solve(BS, LF, RF, self.q0)

            if visualize:
                v.display(posture)
                sleep(1)

            pin.forwardKinematics(self.model, self.data, posture)
            pin.updateFramePlacements(self.model, self.data)

            outLF = self.data.oMf[self.LF_id]
            outRF = self.data.oMf[self.RF_id]
            outBS = self.data.oMi[self.Base_id]

            self.assertTrue((LF.homogeneous - outLF.homogeneous < e).all())
            self.assertTrue((RF.homogeneous - outRF.homogeneous < e).all())
            self.assertTrue((BS.homogeneous - outBS.homogeneous < e).all())

        # self.assertTrue(out)

    def test_com_postures(self):
        e = 1e-11
        if visualize:
            v = get_visualizer()

        LF = pin.SE3(np.eye(3), np.array([0.15, -0.18, 0.2]))
        RF = pin.SE3(np.eye(3), np.array([0.3, -0.38, 0.2]))
        CoM = np.array([0, 0, 0.8])

        for i in range(20):
            self.biped.set_com_from_waist(self.q0)

            t0 = time()
            posture = self.biped.solve(CoM, LF, RF, self.q0, e, i)
            t1 = time()
            if visualize:
                v.display(posture)
                sleep(1)

            pin.forwardKinematics(self.model, self.data, posture)
            pin.updateFramePlacements(self.model, self.data)

            outLF = self.data.oMf[self.LF_id]
            outRF = self.data.oMf[self.RF_id]

            self.assertTrue((LF.homogeneous - outLF.homogeneous < e).all())
            self.assertTrue((RF.homogeneous - outRF.homogeneous < e).all())

            if (pin.centerOfMass(self.model, self.data, posture) - CoM < e).all():
                break

        self.assertTrue(
            (pin.centerOfMass(self.model, self.data, posture) - CoM < e).all()
        )
        print(
            (
                "\nAn error smaller than {} in the"
                + " CoM was obtained with {} iterations"
                + "which are computed in {} seconds"
            ).format(e, i, t1 - t0)
        )
        self.assertTrue(i < 20)


if __name__ == "__main__":

    unittest.main()

    if debug == True:
        sleep(5)
        o = TestBipedIG()
        o.setUp()

        LF = pin.SE3(np.eye(3), np.array([-0.1, 0.185, 0]))
        RF = pin.SE3(np.eye(3), np.array([0.0, -0.58, 0]))
        BS = pin.SE3(np.eye(3), np.array([0, 0, 0.7]))

        #### From computation:

        posture = o.biped.solve(BS, LF, RF, o.q0)
        pin.forwardKinematics(o.model, o.data, posture)
        pin.updateFramePlacements(o.model, o.data)

        hip_fr_ankle = o.data.oMi[2].translation - o.data.oMi[6].translation
        print("hip_from_ankle gotten: ", hip_fr_ankle)

        Q7 = np.arctan2(hip_fr_ankle[1], hip_fr_ankle[2])

        print("q7 gotten: ", Q7)

        C_ = np.linalg.norm(hip_fr_ankle[[0, 2]])

        print("C_ gotten: ", C_)

        Q5 = posture[10]
        C5 = np.cos(Q5)
        print("C5 gotten: ", C5)
        print("Q5_ gotten: ", Q5)

        ### Repeating step by step
        ## 1 The norm of hip_from_ankle are differnet
        ## 2 The angles q7 are the same.
        ### The error must be on q5, check the computation of c_.

        ee = LF
        ll_settings = o.biped.get_left_leg_settings()

        hip_ = BS.translation + BS.rotation @ ll_settings.hip_from_waist
        ankle_ = ee.translation + ee.rotation @ ll_settings.ankle_from_foot

        hip_from_ankle = ee.rotation.T @ (hip_ - ankle_)
        print("hip_from_ankle should be: ", hip_from_ankle)

        q7 = np.arctan2(hip_from_ankle[1], hip_from_ankle[2])

        print("q7 should be: ", q7)

        c_ = np.linalg.norm(hip_from_ankle[[0, 2]])
        a_ = np.abs(ll_settings.knee_from_hip[2])
        b_ = np.abs(ll_settings.ankle_from_knee[2])

        print("c_ should be: ", c_)

        c5 = 0.5 * (c_ * c_ - a_ * a_ - b_ * b_) / (a_ * b_)
        q5 = np.arccos(c5)

        print("c5 should be: ", c5)
        print("q5 should be: ", q5)
