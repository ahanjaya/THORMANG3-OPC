#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np

from pynput.mouse import Listener

def on_click(x, y, button, pressed):
    if pressed:
        print ("Mouse clicked")

def on_scroll(x, y, dx, dy):
    print ("Mouse scrolled")

with Listener(on_click=on_click, on_scroll=on_scroll) as listener:
    listener.join()