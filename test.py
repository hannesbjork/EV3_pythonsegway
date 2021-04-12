#!/usr/bin/env micropython
import time
from time import sleep

import ev3dev.ev3
from ev3dev.ev3 import *

import math

leftWheel = ev3dev.ev3.LargeMotor('outD')
rightWheel = ev3dev.ev3.LargeMotor('outA')

sensorACG = ev3dev.ev3.Sensor(address="in2")
sensorACG.mode = "TILT"

samp_time = 75; #in ms

theta_star = 40

U_MAX = 90
U_MIN = -90

g = 9.816

K = 20
D = 1

#Timed sampling test
next_time = int(round(time.time() * 1000)) + samp_time
while True:

    sensorACG.mode = "TILT"
    theta = sensorACG.value(2) - theta_star

    sensorACG.mode = "GYRO"
    theta_dot = sensorACG.value(2) - theta_star

    u = g * math.sin(math.radians(theta)) * K - D * theta_dot

    if u > U_MAX:
        u = U_MAX
    elif u < U_MIN:
        u = U_MIN

    leftWheel.run_direct(duty_cycle_sp = u)
    rightWheel.run_direct(duty_cycle_sp = u)

    current_time = int(round(time.time() * 1000))

    if current_time < next_time:
        sleep((next_time-current_time)/1000)
    else:
        print("Lagging behind!")
    
    next_time = next_time + samp_time

motorinput = 50

#######################

#Bang-bang control test
while False: 
    if sensorACG.value(2) < 33:
        leftWheel.run_direct(duty_cycle_sp = -motorinput)
        rightWheel.run_direct(duty_cycle_sp = -motorinput)
    elif sensorACG.value(2) > 33:
        leftWheel.run_direct(duty_cycle_sp = motorinput)
        rightWheel.run_direct(duty_cycle_sp = motorinput)

rightWheel.stop()
leftWheel.stop()

#Sensor test
while False:

    sensorACG.mode = "TILT"
    print("tilt-x "+str(sensorACG.value(0)))
    print("tilt-y "+str(sensorACG.value(1)))
    print("tilt-z "+str(sensorACG.value(2)))

    sensorACG.mode = "ACCEL"
    print("accel-x: "+str(sensorACG.value(0)))
    print("accel-y: "+str(sensorACG.value(1)))
    print("accel-z: "+str(sensorACG.value(2)))

    sensorACG.mode = "GYRO"
    print("gyro-x: "+str(sensorACG.value(0)))
    print("gyro-y: "+str(sensorACG.value(1)))
    print("gyro-z: "+str(sensorACG.value(2)))

    sleep(1)

#leftWheel.run_direct(duty_cycle_sp = motorinput)
#rightWheel.run_direct(duty_cycle_sp = motorinput)

sleep(1)

#leftWheel.run_direct(duty_cycle_sp = -motorinput)
#rightWheel.run_direct(duty_cycle_sp = -motorinput)

sleep(1)

#rightWheel.stop()
#leftWheel.stop()