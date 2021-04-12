#!/usr/bin/env python3

import ev3dev.ev3
from ev3dev.ev3 import *

leftWheel = ev3dev.ev3.LargeMotor('outD')
rightWheel = ev3dev.ev3.LargeMotor('outA')

rightWheel.stop()
leftWheel.stop()