#!/usr/bin/env python3
import json
import rospkg
import numpy as np
import matplotlib.pyplot as plt

def checkConsecutive(l): 
    return sorted(l) == list(range(min(l), max(l)+1)) 
      
# Driver Code 
lst = [1, 2, 3, 4, 5] 
print(checkConsecutive(lst)) 

lst = [4,  5, 11, 12] 
print(checkConsecutive(lst)) 
