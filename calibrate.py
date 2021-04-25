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

#Init gyro
gyroSensor = ev3.Sensor(address="in3")
gyroSensorValueRaw  = open(str(gyroSensor._path) + "/value0", "rb") 

############################################################### Functions

def get_time():
    return int(round(time.time() * 1000))

def get_read(infile):
    infile.seek(0)    
    return(int(infile.read().decode().strip()))

def get_bodyangspeed(): 
    gyroRateRaw = get_read( gyroSensorValueRaw)
    return gyroRateRaw

############################################################### Variables

#Count loops
loops = 0
max_loops = 50

#Total calib
GYRO_OFFSET = 0

#Time variables
sample_time = 30 #in ms
sample_time_s = sample_time/1000 #in s

#Set first time
next_time = get_time() + sample_time

############################################################### Calibration loop
while loops < max_loops:

    bodyAngSpeed = get_bodyangspeed()

    GYRO_OFFSET = GYRO_OFFSET + bodyAngSpeed

    print(bodyAngSpeed)

    loops = loops + 1

    #Loop timing
    diff_time = next_time - get_time()
    if diff_time > 0:
        sleep(diff_time/1000)
    else:
        print("Missed deadline: "+str(diff_time)+"ms")
        
    next_time = next_time + sample_time

GYRO_OFFSET = GYRO_OFFSET/max_loops
print(GYRO_OFFSET)

CALIBRATION_PATH = "data/calibration_data.txt"
calib = open(CALIBRATION_PATH,'w') 
calib.write(str(GYRO_OFFSET)) 
calib.close()