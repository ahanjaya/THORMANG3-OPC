#!/usr/bin/env python3
import json
import rospkg
import numpy as np
import matplotlib.pyplot as plt


P = np.ones((100,3))
print(P)
print(P.shape)

P[:,:] = [255, 0, 0]
print(P)


# N = np.zeros((100,3))
# print(N)
# print(N.shape)



# O = np.vstack((P,N))
# print(O)
# print(O.shape)

# O = np.vstack((O,P))
# print(O)
# print(O.shape)


# arr = np.empty((0,3), int)
# arr = np.append(arr, np.array([[1,2,3]]), axis=0)
# arr = np.append(arr, np.array([[1,2,3]]), axis=0)

# print(arr)
# print(arr.shape)