#!/usr/bin/env python3

import os
from gtts import gTTS
from io import BytesIO
mp3_fp = BytesIO()
tts = gTTS('hello', 'en')
tts.write_to_fp(mp3_fp)

os.system("mpg321 {}".format(mp3_fp))