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

    def test_distributor(self):

        self.dyn.distributeForce(
            np.array([0, 0, 0]), np.array([1, 0, 0]), self.dyn.data().com[0]
        )
        leftSole = self.dyn.getContact("left_sole")
        rightSole = self.dyn.getContact("right_sole")

        print(leftSole.appliedForce()[:])
        print(rightSole.appliedForce()[:])
        print(leftSole.appliedForce()[:] + rightSole.appliedForce()[:])


if __name__ == "__main__":
    unittest.main()
