#!/usr/bin/env python3
import time
import ev3dev.ev3

def FastRead(file):
    file.seek(0)
    return file.read()

gyroSensor          = ev3dev.ev3.Sensor(address="in2")
gyroSensorValueRawG  = open(gyroSensor._path + "/value0", "r")
accelSensor         = ev3dev.ev3.Sensor(address="in3")
accelSensorValueRawz1  = open(accelSensor._path + "/value2", "r")
accelSensorValueRawz2  = open(accelSensor._path + "/value5", "r")
while True:
    ti=time.time()
    
    gyro = gyroSensor.value()                           #reading gyro value
    accelraw =  4*accelSensor.value(n=2) + accelSensor.value(n=5)
    tf=time.time()
    dt=tf-ti
    print(dt)
  
