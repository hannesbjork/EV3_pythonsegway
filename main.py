#!/usr/bin/env python3

#Imports
import ev3dev.ev3
from ev3dev.ev3 import *
import math
import time
from time import sleep
import struct

#Init motors
wheel_left = ev3dev.ev3.LargeMotor('outD')
wheel_right = ev3dev.ev3.LargeMotor('outA')

#Init sensor
sensor_ACG = ev3dev.ev3.Sensor(address="in2")
sensor_ACG.mode = "ALL"
sleep(0.005)

#Functions
def get_time():
    return int(round(time.time() * 1000))

def get_sensordata():
    (ang, angvel) = struct.unpack('<2xb18xh', sensor_ACG.bin_data())
    return ((ang-40), angvel)

def get_angvel():
    sensor_ACG.mode = "GYRO"
    return sensor_ACG.value(2)

def set_output(u):
    wheel_left.run_direct(duty_cycle_sp = u)
    wheel_right.run_direct(duty_cycle_sp = u)


#Limit variables
U_MAX = 90 #in % of max
U_MIN = -90 #in -% of max

#Control variables
sample_time = 50; #in ms

g = 9.816 #gravity acceleration

P = 0.5 #P-constant
D = 0.1 #D-constant

K = 0.01

#Control loop
next_time = get_time() + sample_time
u = 0 #TODO
theta_dot_guess = 0 #TODO
while True:

    #Get sensor values
    (theta, theta_dot) = get_sensordata()
    print(str(theta_dot) + "  "+str(theta_dot_guess) )#TODO

    #Control algorithm
    u_old = u #TODO
    u = -(47.4204 * theta + 10.8530 * theta_dot)*K
    theta_dot_guess = u-u_old #TODO

    #Saturation
    if u > U_MAX:
        u = U_MAX
    elif u < U_MIN:
        u = U_MIN

    #Set output signal
    set_output(u)

    #Loop timing
    diff_time = next_time - get_time()
    if diff_time > 0:
        sleep(diff_time/1000)
    else:
        print("Missed deadline: "+str(diff_time)+"ms")
    
    next_time = next_time + sample_time

