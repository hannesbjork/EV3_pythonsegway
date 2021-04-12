#!/usr/bin/env python3
import time
from time import sleep
import ev3dev.ev3
import struct


leftWheelA           = ev3dev.ev3.LargeMotor('outA')
rightWheelD          = ev3dev.ev3.LargeMotor('outD')

portACG = LegoPort(address='ev3-ports:in2')
portACG.mode = 'nxt-i2c'

sensorACG = Sensor(address='ev3-ports:in2:i2c17')

motorinput = 20

#######################

sensorACG.mode = "TILT"
print("tilt-x "+sensorACG.value(0))
print("tilt-y "+sensorACG.value(1))
print("tilt-z "+sensorACG.value(2))

sensorACG.mode = "ACCEL"
print("accel-x: "+sensorACG.value(0))
print("accel-y: "+sensorACG.value(1))
print("accel-z: "+sensorACG.value(2))

sensorACG.mode = "GYRO"
print("gyro-x: "+sensorACG.value(0))
print("gyro-y: "+sensorACG.value(1))
print("gyro-z: "+sensorACG.value(2))

leftWheelA.run_direct(duty_cycle_sp = motorinput)
rightWheelD.run_direct(duty_cycle_sp = motorinput)

sleep(1)

leftWheelA.run_direct(duty_cycle_sp = -motorinput)
rightWheelD.run_direct(duty_cycle_sp = -motorinput)

sleep(1)