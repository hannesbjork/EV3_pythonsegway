#!/usr/bin/env python3

#Imports
import ev3dev.ev3
from ev3dev.ev3 import *
import math
import time
from time import sleep

#Init hardware
leftWheel = ev3dev.ev3.LargeMotor('outD')
rightWheel = ev3dev.ev3.LargeMotor('outA')
sensorACG = ev3dev.ev3.Sensor(address="in2")

#Functions
def get_time():
    return int(round(time.time() * 1000))

def get_ang():
    sensorACG.mode = "TILT"
    return sensorACG.value(2)

def get_angvel():
    sensorACG.mode = "GYRO"
    return sensorACG.value(2)

def set_output(u):
    leftWheel.run_direct(duty_cycle_sp = u)
    rightWheel.run_direct(duty_cycle_sp = u)


#Software variables
U_MAX = 90
U_MIN = -90
samp_time = 75; #in ms
next_time = get_time() + samp_time

#Control variables
theta_star = 40

g = 9.816

K = 20
D = 1

#Control loop
while True:

    #Get input values
    theta = get_ang() - theta_star
    theta_dot = get_angvel() - theta_star

    #Control algorithm
    u = g * math.sin(math.radians(theta)) * K + D * theta_dot

    #Saturation
    if u > U_MAX:
        u = U_MAX
    elif u < U_MIN:
        u = U_MIN

    #Set output signal
    set_output(u)

    #Loop timing
    current_time = int(round(time.time() * 1000))
    if current_time < next_time:
        sleep((next_time-current_time)/1000)
    else:
        print("Lagging behind!")
    
    next_time = next_time + samp_time

