#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 22 16:28:36 2022

@author: nvilla
"""
import aig
import numpy as np
from example_robot_data.path import EXAMPLE_ROBOT_DATA_MODEL_DIR
import unittest
import pinocchio as pin

unittest.util._MAX_LENGTH = 2000


class TestDynaCoM(unittest.TestCase):
    def setUp(self):

        ## CONTACTS ##
        settingsL = aig.Contact6DSettings()
        settingsL.frame_name = "leg_left_sole_fix_joint"
        settingsL.mu = 0.3
        settingsL.gu = 0.4
        settingsL.weights = np.array([1, 1, 1, 1, 1, 1])
        settingsL.half_length = 0.1
        settingsL.half_width = 0.05

        leftSole = aig.Contact6D()
        leftSole.initialize(settingsL)

        settingsR = aig.Contact6DSettings()
        settingsR.frame_name = "leg_right_sole_fix_joint"
        settingsR.mu = 0.3
        settingsR.gu = 0.4
        settingsR.weights = np.array([1, 1, 3, 1, 1, 1])
        settingsR.half_length = 0.1
        settingsR.half_width = 0.05

        rightSole = aig.Contact6D()
        rightSole.initialize(settingsR)

        ### DYNAMICS ###
        dynSettings = aig.DynaCoMSettings()
        dynSettings.urdf = (
            EXAMPLE_ROBOT_DATA_MODEL_DIR + "/talos_data/robots/talos_reduced.urdf"
        )

        d = aig.DynaCoM()
        d.initialize(dynSettings)

        pin.loadReferenceConfigurations(
            d.model(),
            EXAMPLE_ROBOT_DATA_MODEL_DIR + "/talos_data/srdf/talos.srdf",
            False,
        )

        q0 = d.model().referenceConfigurations["half_sitting"]
        v0 = np.zeros(d.model().nv)
        d.computeDynamics(q0, v0, v0, np.zeros(6), True)
        pin.computeAllTerms(d.model(), d.data(), q0, v0)
        pin.updateFramePlacements(d.model(), d.data())

        d.addContact6d(leftSole, "left_sole")
        d.addContact6d(rightSole, "right_sole")

        self.dyn = d
        self.setL = settingsL
        self.setR = settingsR

    def test_contacts(self):

        leftSole = self.dyn.getContact("left_sole")

        self.assertTrue(leftSole.get_settings()["mu"] == self.setL.mu)
        leftSole.set_mu(0.6)
        self.assertTrue(leftSole.get_settings()["mu"] == 0.6)

        self.assertTrue((leftSole.get_settings()["weights"] == self.setL.weights).all())
        leftSole.set_force_weights(np.array([3, 3, 3]))
        self.assertTrue(
            (leftSole.get_settings()["weights"][:3] == np.array([3, 3, 3])).all()
        )
        leftSole.set_torque_weights(np.array([3, 3, 3]))
        self.assertTrue(
            (leftSole.get_settings()["weights"][3:] == np.array([3, 3, 3])).all()
        )

    def test_adjoint(self):

        H1 = pin.SE3(np.eye(3), np.array([0, 1, 0]))
        Adj1 = H1.toActionMatrixInverse().T

        W1l = np.array([0, 0, 1, 0, 0, 0])
        W1o_correct = np.array([0, 0, 1, 1, 0, 0])

        self.assertTrue((Adj1 @ W1l == W1o_correct).all())

        H2 = pin.SE3(pin.utils.rotate("y", 0.3), np.array([0, 1, 0]))
        Adj2 = H2.toActionMatrixInverse().T

        W2l = np.array([0, 0, 0, 1, 0, 0])
        W2o_correct = np.hstack([0, 0, 0, H2.rotation @ [1, 0, 0]])

        self.assertTrue((Adj2 @ W2l == W2o_correct).all())

    def test_distributor(self):

        # single contact

        self.dyn.deactivateContact6d("right_sole")
        data = self.dyn.data()
        oMf = data.oMf[self.dyn.getContact("left_sole").get_frame_id()]
        com = oMf.translation

        correct_lW = np.array([0, 0, 1, 0, 0, 0])  # chosen to not produce CoM torque
        oW = oMf.toActionMatrixInverse().T @ correct_lW

        self.dyn.distributeForce(oW[:3], oW[3:], com)

        lW = self.dyn.getContact("left_sole").appliedForce()
        self.assertTrue((np.abs(lW - correct_lW) < 1e-4).all())

        # double contact

        self.dyn.activateContact6d("right_sole")
        oWd = np.array([0, 0, 10, 0, 0, 0])

        self.dyn.distributeForce(oWd[:3], oWd[3:], com)

        lSW = self.dyn.getContact("left_sole").appliedForce()
        rSW = self.dyn.getContact("right_sole").appliedForce()

        print(lSW)
        print(rSW)
        self.assertTrue((np.abs(lSW[:3] + rSW[:3] - oWd[:3]) < 0.1).all())
        self.assertTrue(lSW[2] / 4 > rSW[2])


if __name__ == "__main__":
    unittest.main()
