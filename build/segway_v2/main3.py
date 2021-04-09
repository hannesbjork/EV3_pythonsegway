#!/usr/bin/env python3
import time
from time import sleep
import ev3dev.ev3

tau = 0
angle = 0
gyro_offset = 2971
acc_scale = 2.841*2
gyro_scale = 0.2084
dt = 0.03
direction = 0

gyroSensor          = ev3dev.ev3.Sensor(address="in2")
accelSensor         = ev3dev.ev3.Sensor(address="in3")

leftWheelA           = ev3dev.ev3.LargeMotor('outA')
leftWheelB           = ev3dev.ev3.LargeMotor('outB')
rightWheelC          = ev3dev.ev3.LargeMotor('outC')
rightWheelD          = ev3dev.ev3.LargeMotor('outD')

motorEncoderStart   = rightWheelC.position 
loop = True
motor=True

ts = time.time()

while loop:

                              #reading gyro value
    
    gyroSensorValueRawG  = open(gyroSensor._path + "/value0", "r")
    accelSensorValueRawz  = open(accelSensor._path + "/value2", "r")
    accelSensorValueRawz2  = open(accelSensor._path + "/value5", "r")
    gyro = int(gyroSensorValueRawG.read()) 
    accelraw =  4*accelSensorValueRawz.read() + accelSensorValueRawz2.read()
    tn = time.time()
    tdiff = tn-ts
  
    if tdiff < dt:
        time.sleep(dt - tdiff)
    ts=time.time()
    print(tdiff)