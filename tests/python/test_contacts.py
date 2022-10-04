#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 22 16:28:36 2022

@author: nvilla
"""
import aig
import numpy as np
from example_robot_data.path import EXAMPLE_ROBOT_DATA_MODEL_DIR


settings = aig.Contact6DSettings()
settings.frame_name = "leg_left_sole_fix_joint"
settings.mu = 1
settings.gu = 1
settings.weights = np.array([1, 1, 1, 1, 1, 1])
settings.half_length = 0.00005
settings.half_width = 0.00005

contact = aig.Contact6D()
contact.initialize(settings)

print("fri_A: \n", contact.fri_A())
print("fri_b: \n", contact.fri_b())
print("uni_A: \n", contact.uni_A())
print("uni_b: \n", contact.uni_b())
print("reg_A: \n", contact.reg_A())
print("reg_b: \n", contact.reg_b())
print("NE_A: \n", contact.NE_A())

dynSettings = aig.DynaCoMSettings()
dynSettings.urdf = (
    EXAMPLE_ROBOT_DATA_MODEL_DIR + "/talos_data/robots/talos_reduced.urdf"
)

d = aig.DynaCoM()
d.initialize(dynSettings)
d.addContact6d(contact, "U")

# d.removeContact6d("U")

d.addContact6d(contact, "G")
d.activateContact6d("G")


d.deactivateContact6d("G")

d.distributeForce(np.array([0, 0, 1]), np.array([0, 0, 0]), np.array([0, 0, 1]))
