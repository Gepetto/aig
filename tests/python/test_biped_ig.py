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

unittest.util._MAX_LENGTH = 2000


settings = aig.BipedIGSettings.makeSettingsFor(
    EXAMPLE_ROBOT_DATA_MODEL_DIR + "/talos_data", "talos"
)
biped = aig.BipedIG()
biped.initialize(settings)

q0 = biped.getQ0()
LF = pin.SE3(np.eye(3), np.array([0, 0.2, 0]))
RF = pin.SE3(np.eye(3), np.array([0, -0.2, 0]))
com = np.array([0, 0, 1])
posture = np.zeros(biped.model().nq)

print(biped.get_com_from_waist())
biped.correctCoMfromWaist(com, LF, RF, q0)
print(biped.get_com_from_waist())

biped.solve(com, LF, RF, q0, posture)

##TODO:
#### the lvalue `posture` in the function `solve` is not working.
##   Noone in internet seems to have this problem. Ask about it.

# class TestDynaCoM(unittest.TestCase):
#     def setUp(self):
