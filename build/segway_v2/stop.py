#!/usr/bin/env python3

import ev3dev.ev3


rightWheelD          = ev3dev.ev3.LargeMotor('outD')
leftWheelA           = ev3dev.ev3.LargeMotor('outA')
rightWheelC          = ev3dev.ev3.LargeMotor('outC')
leftWheelB           = ev3dev.ev3.LargeMotor('outB')

rightWheelC.stop(stop_action="coast")
leftWheelA.stop(stop_action="coast")
rightWheelD.stop(stop_action="coast")
leftWheelB.stop(stop_action="coast")
