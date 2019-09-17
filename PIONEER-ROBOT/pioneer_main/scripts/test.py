#!/usr/bin/env python3
import json
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from gtts import gTTS 
import os 


mytext = 'my name is thormang bear'
# mytext = '歡迎光臨, 國立臺灣師範大學 機器人中心 我叫 許哲函'
language = 'en-us'
# language = 'zh-tw'
myobj = gTTS(text=mytext, lang=language, slow=False) 
myobj.save("welcome.mp3") 
os.system("mpg321 welcome.mp3") 

# import pyttsx3
# engine = pyttsx3.init() # object creation

# """ RATE"""
# rate = engine.getProperty('rate')   # getting details of current speaking rate
# print (rate)                        #printing current voice rate
# engine.setProperty('rate', 125)     # setting up new voice rate


# """VOLUME"""
# volume = engine.getProperty('volume')   #getting to know current volume level (min=0 and max=1)
# print (volume)                          #printing current volume level
# engine.setProperty('volume',1.0)    # setting up volume level  between 0 and 1

# """VOICE"""
# voices = engine.getProperty('voices')       #getting details of current voice
# print(len(voices))
# engine.setProperty('voice', voices[0].id)  #changing index, changes voices. o for male
# # engine.setProperty('voice', voices[1].id)   #changing index, changes voices. 1 for female

# engine.say("Hello World!")
# engine.say('My current speaking rate is ' + str(rate))
# engine.runAndWait()
# engine.stop()

# engine.save_to_file('halo.mp3', '~/Music')
