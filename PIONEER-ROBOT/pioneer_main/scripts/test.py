#!/usr/bin/env python3
import json
import rospkg
import numpy as np
import matplotlib.pyplot as plt

left  = np.matrix([ [0, 153], [58, 269] ])
right = np.matrix([ [324, -8], [382, 107] ])

print('left: ', left)
print('right: ', right)

idx_P1 = np.argmin(left[:,1])
idx_P3 = np.argmax(left[:,1])

P1 = left[idx_P1]
P3 = left[idx_P3]

idx_P2 = np.argmin(right[:,1])
idx_P4 = np.argmax(right[:,1])

P2 = right[idx_P2]
P4 = right[idx_P4]


print('P1 ', P1)
print('P3 ', P3)

print('P2 ', P2)
print('P4 ', P4)