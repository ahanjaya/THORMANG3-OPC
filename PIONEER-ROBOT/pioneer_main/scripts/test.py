#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np

a = np.arange(-5, 6, 1)
a = np.delete(a, np.argwhere(a==0))



print(a)
print(a.shape)
