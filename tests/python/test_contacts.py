#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 22 16:28:36 2022

@author: nvilla
"""
import unittest
from time import sleep

import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from example_robot_data.path import EXAMPLE_ROBOT_DATA_MODEL_DIR

import aig

unittest.util._MAX_LENGTH = 2000

debug = False
visualize = True


class TestDynaCoM(unittest.TestCase):
    def setUp(self):
        # CONTACTS ##
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
        settingsR.weights = np.array([1, 1, 1, 1, 1, 1])
        settingsR.half_length = 0.1
        settingsR.half_width = 0.05

        rightSole = aig.Contact6D()
        rightSole.initialize(settingsR)

        # DYNAMICS ###
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

        # BIPED IG to compute postures
        settings = aig.BipedIGSettings.makeSettingsFor(
            EXAMPLE_ROBOT_DATA_MODEL_DIR + "/talos_data", "talos"
        )
        biped = aig.BipedIG()
        biped.initialize(settings)

        self.dyn = d
        self.setL = settingsL
        self.setR = settingsR
        self.q0 = q0
        self.biped = biped

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
        self.assertTrue(
            list(self.dyn.getActiveContacts()) == ["left_sole", "right_sole"]
        )
        self.dyn.deactivateContact6d("left_sole")
        self.assertTrue("left_sole" not in list(self.dyn.getActiveContacts()))

        self.dyn.deactivateContact6d("right_sole")
        self.assertTrue(not list(self.dyn.getActiveContacts()))

        self.dyn.activateContact6d("left_sole")
        self.assertTrue("left_sole" in list(self.dyn.getActiveContacts()))

        self.dyn.activateContact6d("right_sole")
        self.assertTrue(
            list(self.dyn.getActiveContacts()) == ["left_sole", "right_sole"]
        )
        self.dyn.removeContact6d("right_sole")
        self.assertTrue("right_sole" not in list(self.dyn.getActiveContacts()))

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

    def test_distribution_on_single_contact(self):
        self.dyn.deactivateContact6d("right_sole")
        pin.updateFramePlacements(self.dyn.model(), self.dyn.data())
        data = self.dyn.data()
        oMs = data.oMf[self.dyn.getContact("left_sole").get_frame_id()]
        com = oMs.translation + np.array([0, -0.2, 1])

        cMo = pin.SE3(np.eye(3), -com)
        cXs = (cMo.act(oMs)).toActionMatrixInverse().T

        sMs = pin.SE3(oMs.rotation.T, np.zeros(3))
        correct_lW = sMs.toActionMatrixInverse().T @ np.array([0, 0, 10000, 0, 0, 0])

        cW = cXs @ correct_lW
        self.assertTrue((cW - np.array([0, 0, 10000, 2000, 0, 0]) < 1e-3).all())

        self.dyn.distributeForce(cW[:3], cW[3:], com)

        lW = self.dyn.getContact("left_sole").appliedForce()
        self.assertTrue((np.abs(lW - correct_lW) < 1e-4).all())

    def test_distribution_on_double_contact(self):
        com = np.array([0, 0, 2])
        cW = np.array([0, 0, 1, 0.01, 0, 0])

        self.dyn.distributeForce(cW[:3], cW[3:], com)
        self.assertTrue(
            (
                self.dyn.getContact("left_sole").appliedForce()[:3]
                + self.dyn.getContact("right_sole").appliedForce()[:3]
                - cW[:3]
                < 1e-4
            ).all()
        )

        self.assertTrue(
            self.dyn.getContact("left_sole").appliedForce()[2]
            > self.dyn.getContact("right_sole").appliedForce()[2]
        )

    def test_dynamic_distribution(self):
        LF = self.dyn.getContact("left_sole").get_pose()
        RF = self.dyn.getContact("right_sole").get_pose()
        N_cycles = 5
        N = 500

        self.dyn.getContact("left_sole").set_force_weights(np.array([1e-5, 1e-5, 1e-5]))
        self.dyn.getContact("right_sole").set_force_weights(
            np.array([1e-5, 1e-5, 1e-5])
        )

        time = np.linspace(0, N, N) * N_cycles * 2 * np.pi / N
        A = -(LF.translation[1] - RF.translation[1]) * 1.97 / 2
        Dt = 0.01

        CoP_traj = []
        CoM_traj = []
        force_traj = []
        torque_traj = []
        LF_wrench_traj = []
        RF_wrench_traj = []

        for t in time:
            tb = t - Dt
            ta = t + Dt

            Bb = np.array([0, A * np.sin(tb), 1.05])
            B = np.array([0, A * np.sin(t), 1.05])
            Ba = np.array([0, A * np.sin(ta), 1.05])

            q, dq, ddq = self.biped.solve(
                [pin.SE3(np.eye(3), Bb), pin.SE3(np.eye(3), B), pin.SE3(np.eye(3), Ba)],
                [LF, LF, LF],
                [RF, RF, RF],
                self.q0,
                Dt,
            )

            self.dyn.computeDynamics(
                q, dq, ddq, np.zeros(6), True
            )  # no distributed foreces
            self.dyn.getGroundCoMForce()
            com = pin.centerOfMass(self.dyn.model(), self.dyn.data(), q)

            self.dyn.distributeForce(
                self.dyn.getGroundCoMForce(), self.dyn.getGroundCoMTorque(), com
            )

            CoP_traj.append(self.dyn.getCoP().copy())
            CoM_traj.append(com.copy())
            force_traj.append(self.dyn.getGroundCoMForce().copy())
            torque_traj.append(self.dyn.getGroundCoMTorque().copy())
            LF_wrench_traj.append(
                self.dyn.getContact("left_sole").appliedForce().copy()
            )
            RF_wrench_traj.append(
                self.dyn.getContact("right_sole").appliedForce().copy()
            )

        CoP_xy = np.vstack(CoP_traj)
        CoM_xy = np.vstack(CoM_traj)
        force_xyz = np.vstack(force_traj)
        LF_wrench = np.vstack(LF_wrench_traj)
        RF_wrench = np.vstack(RF_wrench_traj)

        Correct_left_z_force = np.array(
            [
                4.50192949e02,
                3.98974165e02,
                3.47957988e02,
                2.97346437e02,
                2.47339945e02,
                1.98136571e02,
                1.49931219e02,
                1.02914870e02,
                5.72738206e01,
                1.31889523e01,
                9.76705160e-15,
                4.46585262e-16,
                0.00000000e00,
                0.00000000e00,
                3.79271365e-14,
                -4.38330328e-14,
                -5.15596220e-14,
                -1.12562859e-13,
                6.11975133e-14,
                8.34706837e-24,
                0.00000000e00,
                -8.90456187e-24,
                -7.45561798e-14,
                7.62336837e-14,
                -1.54458103e-13,
                -7.75383463e-14,
                0.00000000e00,
                0.00000000e00,
                7.43511443e-14,
                -7.19310612e-14,
                -9.46924668e-24,
                9.12962997e-24,
                -6.07316609e-14,
                -8.89237429e-16,
                8.30132347e-16,
                -7.63555479e-16,
                3.72298732e-14,
                2.99716823e-14,
                -2.27411413e-14,
                1.39899018e-14,
                0.00000000e00,
                1.75223795e01,
                6.17708362e01,
                1.07557650e02,
                1.54701366e02,
                2.03015185e02,
                2.52307700e02,
                3.02383656e02,
                3.53044722e02,
                4.04090269e02,
                4.55318165e02,
                5.06525564e02,
                5.57509707e02,
                6.08068717e02,
                6.58002391e02,
                7.07112991e02,
                7.55206018e02,
                8.02090978e02,
                8.47582139e02,
                8.85563634e02,
                8.85559774e02,
                8.85555974e02,
                8.85552295e02,
                8.85548792e02,
                8.85545518e02,
                8.85542515e02,
                8.85539820e02,
                8.85537458e02,
                8.85535445e02,
                8.85533785e02,
                8.85532468e02,
                8.85531470e02,
                8.85530758e02,
                8.85530295e02,
                8.85530044e02,
                8.85529981e02,
                8.85530099e02,
                8.85530410e02,
                8.85530944e02,
                8.85531737e02,
                8.85532828e02,
                8.85534247e02,
                8.85536012e02,
                8.85538130e02,
                8.85540594e02,
                8.85543385e02,
                8.85546474e02,
                8.85549821e02,
                8.85553382e02,
                8.85557104e02,
                8.85560929e02,
                8.78497745e02,
                8.34091978e02,
                7.88163488e02,
                7.40896375e02,
                6.92477878e02,
                6.43099770e02,
                5.92957605e02,
                5.42249943e02,
                4.91177571e02,
                4.39942714e02,
                3.88748240e02,
                3.37796866e02,
                2.87290357e02,
                2.37428733e02,
                1.88409486e02,
                1.40426796e02,
                9.36707651e01,
                4.83266679e01,
                4.57060219e00,
                7.97145477e-15,
                -4.64588587e-16,
                2.45924162e-14,
                0.00000000e00,
                -3.93057222e-14,
                0.00000000e00,
                2.34564699e-14,
                5.72979007e-14,
                6.21105167e-14,
                -6.62969445e-14,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                1.35221435e-23,
                -1.38200191e-23,
                -1.40446547e-23,
                -7.70023522e-14,
                0.00000000e00,
                0.00000000e00,
                -1.42733757e-13,
                6.81500529e-14,
                6.42831004e-14,
                -5.97813470e-14,
                -5.55406837e-14,
                -4.89472791e-14,
                0.00000000e00,
                -6.74793164e-16,
                2.84592408e-14,
                2.06062154e-14,
                -1.27109242e-14,
                3.54840260e-15,
                2.62404906e01,
                7.08108541e01,
                1.16883746e02,
                1.64276585e02,
                2.12801590e02,
                2.62266523e02,
                3.12475452e02,
                3.63229524e02,
                4.14327746e02,
                4.65567779e02,
                5.16746731e02,
                5.67661955e02,
                6.18111846e02,
                6.67896633e02,
                7.16819161e02,
                7.64685672e02,
                8.11306566e02,
                8.56497149e02,
                8.85562860e02,
                8.85559007e02,
                8.85555227e02,
                8.85551578e02,
                8.85548118e02,
                8.85544895e02,
                8.85541951e02,
                8.85539320e02,
                8.85537027e02,
                8.85535085e02,
                8.85533495e02,
                8.85532244e02,
                8.85531306e02,
                8.85530647e02,
                8.85530229e02,
                8.85530017e02,
                8.85529990e02,
                8.85530145e02,
                8.85530498e02,
                8.85531081e02,
                8.85531931e02,
                8.85533085e02,
                8.85534572e02,
                8.85536408e02,
                8.85538596e02,
                8.85541127e02,
                8.85543980e02,
                8.85547124e02,
                8.85550518e02,
                8.85554116e02,
                8.85557863e02,
                8.85561700e02,
                8.69748485e02,
                8.25022186e02,
                7.78811235e02,
                7.31298710e02,
                6.82672817e02,
                6.33126146e02,
                5.82854915e02,
                5.32058190e02,
                4.80937111e02,
                4.29694094e02,
                3.78532042e02,
                3.27653545e02,
                2.77260080e02,
                2.27551227e02,
                1.78723877e02,
                1.30971454e02,
                8.44831555e01,
                3.94432010e01,
                0.00000000e00,
                9.32037162e-15,
                1.86412754e-14,
                2.61536886e-14,
                0.00000000e00,
                4.06628714e-14,
                -4.79173883e-14,
                8.22920615e-14,
                -1.16619276e-13,
                0.00000000e00,
                -1.32869620e-23,
                -7.04709030e-14,
                -1.43647417e-23,
                1.50944759e-23,
                -1.52527317e-23,
                -7.74352277e-14,
                -7.74695983e-14,
                -7.68169443e-14,
                -7.54798470e-14,
                0.00000000e00,
                -7.07761730e-14,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                -5.27000772e-14,
                0.00000000e00,
                0.00000000e00,
                3.43883314e-14,
                0.00000000e00,
                1.89791914e-14,
                -1.01752130e-14,
                0.00000000e00,
                3.50257903e01,
                7.99109921e01,
                1.26262657e02,
                1.73897105e02,
                2.22625604e02,
                2.72255115e02,
                3.22589061e02,
                3.73428097e02,
                4.24570898e02,
                4.75814949e02,
                5.26957345e02,
                5.77795584e02,
                6.28128364e02,
                6.77756373e02,
                7.26483076e02,
                7.74115486e02,
                8.20464923e02,
                8.65347764e02,
                8.85562087e02,
                8.85558244e02,
                8.85554485e02,
                8.85550870e02,
                8.85547453e02,
                8.85544282e02,
                8.85541399e02,
                8.85538834e02,
                8.85536611e02,
                8.85534739e02,
                8.85533218e02,
                8.85532032e02,
                8.85531153e02,
                8.85530545e02,
                8.85530171e02,
                8.85529997e02,
                8.85530006e02,
                8.85530199e02,
                8.85530595e02,
                8.85531228e02,
                8.85532136e02,
                8.85533355e02,
                8.85534911e02,
                8.85536817e02,
                8.85539075e02,
                8.85541673e02,
                8.85544587e02,
                8.85547784e02,
                8.85551223e02,
                8.85554855e02,
                8.85558625e02,
                8.85562473e02,
                8.60930594e02,
                8.15892989e02,
                7.69406902e02,
                7.21656496e02,
                6.72830913e02,
                6.23123531e02,
                5.72731198e02,
                5.21853458e02,
                4.70691771e02,
                4.19448714e02,
                3.68327190e02,
                3.17529630e02,
                2.67257197e02,
                2.17708993e02,
                1.69081278e02,
                1.21566693e02,
                7.53534980e01,
                3.06248287e01,
                2.33494663e-15,
                1.18461719e-14,
                0.00000000e00,
                5.86026490e-16,
                3.44395356e-14,
                0.00000000e00,
                -8.10914199e-16,
                -8.34116343e-14,
                -5.92969233e-14,
                1.44264197e-23,
                -1.54432328e-23,
                -7.10748355e-14,
                0.00000000e00,
                1.74113002e-23,
                -1.78315891e-23,
                -1.83034450e-23,
                -7.73939859e-14,
                -7.66041499e-14,
                -7.51307124e-14,
                1.69756529e-23,
                7.01590376e-14,
                -6.66804861e-14,
                -1.25115263e-13,
                5.78068161e-14,
                0.00000000e00,
                4.72901120e-14,
                3.99869899e-14,
                -3.35797353e-14,
                5.60738271e-16,
                -1.78074908e-14,
                -3.82480101e-16,
                3.10124266e-01,
                4.38768854e01,
                8.90698071e01,
                1.35692896e02,
                1.83561401e02,
                2.32485669e02,
                2.82271894e02,
                3.32722880e02,
                3.83638824e02,
                4.34818100e02,
                4.86058052e02,
                5.37155790e02,
                5.87908988e02,
                6.38116681e02,
                6.87580050e02,
                7.36103206e02,
                7.83493964e02,
                8.29564597e02,
                8.74132581e02,
                8.85561314e02,
                8.85557483e02,
                8.85553748e02,
                8.85550169e02,
                8.85546798e02,
                8.85543681e02,
                8.85540859e02,
                8.85538361e02,
                8.85536208e02,
                8.85534407e02,
                8.85532955e02,
                8.85531833e02,
                8.85531011e02,
                8.85530453e02,
                8.85530121e02,
                8.85529984e02,
                8.85530029e02,
                8.85530261e02,
                8.85530702e02,
                8.85531386e02,
                8.85532354e02,
                8.85533638e02,
                8.85535264e02,
                8.85537241e02,
                8.85539568e02,
                8.85542231e02,
                8.85545205e02,
                8.85548454e02,
                8.85551936e02,
                8.85555600e02,
                8.85559390e02,
                8.85563247e02,
                8.52047605e02,
                8.06705835e02,
                7.59951981e02,
                7.11971262e02,
                6.62953727e02,
                6.13093509e02,
                5.62588058e02,
                5.11637365e02,
                4.60443176e02,
                4.09208197e02,
                3.58135301e02,
                3.07426727e02,
                2.57283291e02,
                2.07903589e02,
                1.59483218e02,
                1.12214004e02,
                6.62832403e01,
                2.18729498e01,
                4.44103555e-15,
                6.67698964e-15,
                0.00000000e00,
                -6.02640795e-16,
                3.58448033e-14,
                7.56512887e-16,
                -8.23800260e-16,
                0.00000000e00,
                1.59837096e-23,
                6.46986345e-14,
                1.81878352e-23,
                0.00000000e00,
                7.41393596e-14,
                0.00000000e00,
                7.70847769e-14,
                7.75314711e-14,
                7.72908959e-14,
                -2.03947193e-23,
                0.00000000e00,
                7.24686311e-14,
                6.95155705e-14,
                0.00000000e00,
                0.00000000e00,
                3.96479840e-17,
                5.13036235e-14,
                7.77436549e-16,
                7.05381693e-16,
                3.20913475e-14,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                8.87150246e00,
                5.27923720e01,
                9.82858469e01,
                1.45172966e02,
                1.93267940e02,
                2.42380224e02,
                2.92315271e02,
                3.42875303e02,
                3.93860088e02,
                4.45067731e02,
                4.96295464e02,
                5.47340448e02,
                5.98000566e02,
                6.48075216e02,
                6.97366106e02,
                7.45678026e02,
                7.92819621e02,
                8.38604147e02,
                8.82845799e02,
                8.85560543e02,
                8.85556726e02,
                8.85553018e02,
                8.85549476e02,
                8.85546153e02,
                8.85543092e02,
                8.85540333e02,
                8.85537903e02,
                8.85535820e02,
                8.85534089e02,
                8.85532705e02,
                8.85531645e02,
                8.85530880e02,
                8.85530370e02,
                8.85530079e02,
                8.85529979e02,
                8.85530060e02,
                8.85530331e02,
                8.85530818e02,
                8.85531556e02,
                8.85532585e02,
                8.85533936e02,
                8.85535631e02,
                8.85537679e02,
                8.85540075e02,
                8.85542802e02,
                8.85545834e02,
                8.85549133e02,
                8.85552656e02,
                8.85556350e02,
                8.85560158e02,
                8.85564020e02,
                8.43100927e02,
                7.97462179e02,
                7.50447970e02,
                7.02244542e02,
                6.53042822e02,
                6.03037669e02,
                5.52427103e02,
                5.01411530e02,
                4.50192949e02,
            ]
        )

        self.assertTrue((LF_wrench[:, 2] - Correct_left_z_force < 1e-5).all())

        if visualize:
            figure, ax = plt.subplots(2, 1)

            ax[1].plot(time, CoP_xy[:, 1])
            ax[1].plot(time, CoM_xy[:, 1])

            ax[0].plot(time, force_xyz[:, 2])
            ax[0].plot(time, LF_wrench[:, 2])
            ax[0].plot(time, RF_wrench[:, 2])

    def test_dynamic_computation(self):
        e = 1e-12
        q0 = self.dyn.model().referenceConfigurations["half_sitting"]
        v0 = np.zeros(self.dyn.model().nv)
        self.dyn.computeDynamics(
            q0, v0, v0, np.zeros(6), True
        )  # no distributed foreces

        com_flat = self.dyn.getCoM()
        cop_flat = self.dyn.getCoP()

        self.assertTrue((com_flat[:2] - cop_flat[:2] < e).all())

        self.dyn.computeDynamics(q0, v0, v0, np.zeros(6), False)  # distributed foreces

        com_noflat = self.dyn.getCoM()
        cop_noflat = self.dyn.getCoP()

        self.assertTrue((com_noflat[:2] - cop_noflat[:2] < e).all())
        self.assertTrue((com_flat[:2] - com_noflat[:2] < e).all())
        self.assertTrue((cop_flat[:2] - cop_noflat[:2] < e).all())

        # As the acceleration is zero, the non-linearities are the same for any w
        n_1 = self.dyn.computeNL(1)
        n_9 = self.dyn.computeNL(9)

        self.assertTrue(n_1 == n_9)

        """  The following commented example corresponds to an infeasible case
              ProxSuite produces infeasible solutions in such cases, we have to
              check for infeasibility ourselves and deal with it.  """
        # self.dyn.deactivateContact6d("right_sole")
        # self.dyn.computeDynamics(q0, v0, v0, np.zeros(6), False)


if __name__ == "__main__":
    unittest.main()

    if debug:
        sleep(0.001)
        print("___________________----------------__")

        h = TestDynaCoM()
        h.setUp()
