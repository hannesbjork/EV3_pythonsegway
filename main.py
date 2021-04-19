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
count_per_rot = (wheel_left.count_per_rot + wheel_right.count_per_rot)/2


#Init sensor
sensor_ACG = ev3dev.ev3.Sensor(address="in2")
sensor_ACG.mode = "ALL"
sleep(0.05)

#Functions
def get_time():
    return int(round(time.time() * 1000))

def get_sensordata():
    (ang, angvel) = struct.unpack('<2xb18xh', sensor_ACG.bin_data())
    return ((math.asin(ang/128)*180/math.pi)-40, (angvel*0.00875))

def get_motordata():
    pos = (wheel_left.position + wheel_right.position)/(2*count_per_rot)
    vel = (wheel_left.speed + wheel_right.speed)/(2*count_per_rot)

    return (pos%180-94.6, vel)

def set_output_limit(u, angle, max_angle):
    if abs(angle)>max_angle:
        wheel_left.stop()
        wheel_right.stop()
    else:
        wheel_left.run_direct(duty_cycle_sp = u)
        wheel_right.run_direct(duty_cycle_sp = u)


#Limit variables
U_MAX = 90 #in % of max
U_MIN = -90 #in -% of max

#Control variables
sample_time = 30; #in ms

K = 1

#Control loop
next_time = get_time() + sample_time
while True:

    #Get sensor values
    (psi, psi_dot) = get_sensordata()
    (theta, theta_dot) = get_motordata()

    print(psi)

    #Control algorithm
    u = -(7.3368*theta  +43.2866*theta_dot +1.5090*psi + 3.2005*psi_dot)*K

    #Saturation
    if u > U_MAX:
        u = U_MAX
    elif u < U_MIN:
        u = U_MIN

    #Set output signal with an angle limit
    set_output_limit(u, psi, 40)

    #Loop timing
    diff_time = next_time - get_time()
    if diff_time > 0:
        sleep(diff_time/1000)
    else:
        print("Missed deadline: "+str(diff_time)+"ms")
    
    next_time = next_time + sample_time

