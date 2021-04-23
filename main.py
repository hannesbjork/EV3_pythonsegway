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
sensor_ACG = ev3dev.ev3.Sensor(address="in3")
#sensor_ACG.mode = "ALL" #TODO
sleep(0.05)

#Compensation variables
comp_bodyang=0
comp_wheelang=0

#Limit variables
U_MAX = 100 #in % of max
U_MIN = -100 #in -% of max
ANG_MAX = 40

#Functions
def get_time():
    return int(round(time.time() * 1000))

def get_sensordata():
    #(ang, angvel) = struct.unpack('<2xb18xh', sensor_ACG.bin_data())
    #return ((math.asin(ang/128)*180/math.pi)-comp_bodyang, (angvel*0.00875))
    gyro = sensor_ACG.value() 
    return (gyro-2951)*0.2084

def get_motordata():
    pos = (wheel_left.position + wheel_right.position)/(4)
    vel = (wheel_left.speed + wheel_right.speed)/(4)

    return (-(pos+comp_wheelang), -vel)

def set_output(u, angle):
    if abs(angle)>ANG_MAX:
        wheel_left.stop()
        wheel_right.stop()
    else:
        wheel_left.run_direct(duty_cycle_sp = -u)
        wheel_right.run_direct(duty_cycle_sp = -u)


#Set compensation vars
#comp_bodyang = get_sensordata()[0]
comp_bodyang = get_sensordata()
comp_wheelang = get_motordata()[0]

#Control variables
sample_time = 30 #in ms

K = 0.075 #scalar for the motor output, 7.5V / 100%
K = 0.1

theta_param = -36.1829   #K1
psi_param = -194.3097 #K2 
theta_dot_param = -4.1627  #K3
psi_dot_param = -15.2284 #K4


#theta_param = -1.0035  #K1
#psi_param = 158.7342 #K2 
#theta_dot_param = -1.4785  #K3
#psi_dot_param = -10.4721 #K4

#theta_param = -5.9552 #K1
#psi_param = -38.7172 #K2 
#theta_dot_param = -1.3539  #K3
#psi_dot_param = -2.8635 #K4

#theta_param = -0.0010 #K1
#psi_param = 254.3399 #K2 
#theta_dot_param = -0.0476  #K3
#psi_dot_param = -12.5356 #K4

#theta_param = -7  #K1
#psi_param = 1700 #K2 
#theta_dot_param = -9  #K3
#psi_dot_param = 120 #K4

#Temp vars TODO
psi = 0
theta = 0
psi_dot = 0
theta_dot = 0
psi_old = 0
theta_old = 0
dummy = 0

#Control loop
next_time = get_time() + sample_time
while True:

    #Testing TODO
    psi_old = psi
    theta_old = theta

    #Get sensor values
    
    psi_dot = get_sensordata()
    psi = psi + psi_dot*sample_time/1000
    
    #(psi, psi_dot) = get_sensordata()
    (theta, theta_dot) = get_motordata()

    #Testing TODO
    #psi_dot = ((psi-psi_old)*4 + psi_dot)/5
    #theta_dot = ((theta-theta_old)*2 + theta_dot)/3

    #Debug TODO
    print("psi:"+ str(math.trunc(psi))+" psi*:"+ str(math.trunc(psi_dot))+" theta:"+ str(math.trunc(theta))+" theta*:"+ str(math.trunc(theta_dot)))

    #Control algorithm
    u = K*(theta_dot_param*theta_dot -psi_param*psi - psi_dot_param*psi_dot +theta_param*theta  )
    #u = 0 #Temp TODO

    #Saturation
    if u > U_MAX:
        u = U_MAX
    elif u < U_MIN:
        u = U_MIN

    #Set output signal and give signal so that a limit can be enforced
    set_output(u, psi)

    #Loop timing
    diff_time = next_time - get_time()
    if diff_time > 0:
        sleep(diff_time/1000)
    else:
        print("Missed deadline: "+str(diff_time)+"ms")
        
    next_time = next_time + sample_time

