#!/usr/bin/env python

from arbotix import *
import time

a = ArbotiX(baud=1000000)

while True:
    print a.getPosition(8)
    time.sleep(0.25)
