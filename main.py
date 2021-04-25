#!/usr/bin/env python3

############################################################### Imports

import ev3dev.ev3 as ev3
from ev3dev.ev3 import *
import math
import time
import sys
import importlib
from time import sleep

############################################################### Inits

#Init motors
motorLeft = ev3.LargeMotor('outD')
motorRight = ev3.LargeMotor('outA') 

motorLeft.reset()              
motorRight.reset()
motorLeft.run_direct()
motorRight.run_direct()

motorEncoderLeft    = open(str(motorLeft._path) + "/position", "rb")    
motorEncoderRight   = open(str(motorRight._path) + "/position", "rb")  
    
motorDutyCycleLeft = open(str(motorLeft._path) + "/duty_cycle_sp", "w")
motorDutyCycleRight= open(str(motorRight._path) + "/duty_cycle_sp", "w")

#Init gyro
gyroSensor = ev3.Sensor(address="in3")
gyroSensorValueRaw  = open(str(gyroSensor._path) + "/value0", "rb") 

#Init button
touchSensor = ev3.Sensor(address="in2")

############################################################### Constants

U_MAX = 100 #in % of max
U_MIN = -100 #in -% of max
ANG_MAX = 40
MOTOR_VOLTAGE_PERCENT = 0.75 #7.5V / 100%
RAD_PER_DEG = 3.14159/180 

############################################################### Get latest calibration

CALIBRATION_PATH = "segway_calibration.txt"
calib = open(CALIBRATION_PATH, 'r')

GYRO_OFFSET_raw = calib.readline()
GYRO_OFFSET_raw.replace(" ", "")
GYRO_OFFSET = float(GYRO_OFFSET_raw)

calib.close()

############################################################### Functions

#Read from sensor and motor file
def get_read(infile):
    infile.seek(0)    
    return(int(infile.read().decode().strip()))

#Write to motors    
def set_write(outfile,value):
    outfile.truncate(0)
    outfile.write(str(int(value)))
    outfile.flush() 

#Get time in ms
def get_time():
    return int(round(time.time() * 1000))


def get_bodyangspeed(): 
    gyroRateRaw = get_read( gyroSensorValueRaw)
    gyroRate = (gyroRateRaw - GYRO_OFFSET)*0.2084*RAD_PER_DEG
    return gyroRate

def get_wheelposition():
    motorAngleRaw = (get_read(motorEncoderLeft) + get_read(motorEncoderRight))/2
    motorAngle = motorAngleRaw*RAD_PER_DEG
    return motorAngle

def stop_motors():
    motorLeft.stop()
    motorRight.stop()

def set_duty(motorDutyFilePath, u):

    u_int = -int(round(u*MOTOR_VOLTAGE_PERCENT))

    #Saturation
    if u_int > U_MAX:
        u_int = U_MAX
    elif u_int < U_MIN:
        u_int = U_MIN

    # Apply the signal to the motor
    set_write(motorDutyFilePath, u_int)


############################################################### Control vars

#Time variables
sample_time = 30 #in ms
sample_time_s = sample_time/1000 #in s

#Control parameters
param_wheelang  = -7  #K1
param_bodyang = 1700 #K2 
param_wheelangspeed = -9  #K3
param_bodyangspeed = 120 #K4

#Establish recurring variables
wheelAng_old = 0
wheelAngSpeed = 0
wheelAng = 0
bodyAng = 0

#Set first time
next_time = get_time() + sample_time

Sound.beep()

############################################################### Control loop
while True:

    wheelAng_old = wheelAng
    wheelAng = get_wheelposition()
    bodyAngSpeed = get_bodyangspeed()

    bodyAng = bodyAng + bodyAngSpeed*sample_time_s
    wheelAngSpeed = (wheelAng-wheelAng_old)/sample_time_s

    #Debug TODO
    print("wheelAng:"+ str(math.trunc(wheelAng))+" bodyAng:"+ str(math.trunc(bodyAng))+" wheelAngSpeed:"+ str(math.trunc(wheelAngSpeed))+" bodyAngSpeed:"+ str(math.trunc(bodyAngSpeed)))

    #Control algorithm
    u = ( param_wheelang * wheelAng 
        + param_bodyang * bodyAng 
        + param_wheelangspeed * wheelAngSpeed 
        + param_bodyangspeed * bodyAngSpeed )

    set_duty(motorDutyCycleRight, u)
    set_duty(motorDutyCycleLeft , u)

    #Loop timing
    diff_time = next_time - get_time()
    if diff_time > 0:
        sleep(diff_time/1000)
    else:
        print("Missed deadline: "+str(diff_time)+"ms")
        
    next_time = next_time + sample_time

    #Button reset
    while touchSensor.value():

        motorLeft.reset()              
        motorRight.reset()
        motorLeft.run_direct()
        motorRight.run_direct()

        set_duty(motorDutyCycleRight, 0)
        set_duty(motorDutyCycleLeft , 0)
        bodyAng=0
        time.sleep(0.1)
        next_time = get_time() + sample_time
  

