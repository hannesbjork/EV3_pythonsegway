#!/usr/bin/env python3

import time
from time import sleep
import ev3dev.ev3

leftWheelA           = ev3dev.ev3.LargeMotor('outA')
rightWheelD          = ev3dev.ev3.LargeMotor('outD')


motorinput = 50

leftWheelA.run_direct(duty_cycle_sp = motorinput)
rightWheelD.run_direct(duty_cycle_sp = motorinput)

sleep(1)

leftWheelA.run_direct(duty_cycle_sp = -motorinput)
rightWheelD.run_direct(duty_cycle_sp = -motorinput)

sleep(1)