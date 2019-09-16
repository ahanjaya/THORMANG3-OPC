#!/usr/bin/env python3
import json
import rospkg
import numpy as np
import matplotlib.pyplot as plt

while True:
    key = input("Input key : ")
    key = key.lower()

    if "\\n" in key:
        words = key.split('\\n')
        print(words)
        for word in words:
            for alphabet in word:
                print(alphabet)
            print('enter')
    else:
        for i in key:
            print(i)

