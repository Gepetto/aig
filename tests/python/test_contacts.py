#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 22 16:28:36 2022

@author: nvilla
"""
import aig
import numpy as np

settings = aig.Contact6DSettings()
settings.frame_name = "leg_left_sole_fix_joint"
settings.mu = 7
settings.gu = 9
settings.weights = np.array([1, 2, 3, 4, 5, 6])
settings.half_length = 0.5
settings.half_width = 0.3

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
dynSettings.urdf = "/local/users/nvilla/workspace/install/share/example-robot-data/robots/talos_data/robots/talos_reduced.urdf"

d = aig.DynaCoM()
d.initialize(dynSettings)
d.addContact6d(contact, "U")

d.removeContact6d("U")

d.addContact6d(contact, "G")
d.activateContact6d("G")
d.addContact6d(contact, "A")
d.addContact6d(contact, "C")


# d.deactivateContact6d("G")

d.distributeForce(
    np.array([1, 2, 1]), np.array([0.2, 0.2, 0.1]), np.array([99, 77, 55])
)
