#!/usr/bin/env python3

import time
from time import sleep
import ev3dev.ev3
import math

tau = 0
angle = 0
gyro_offset = 2971
acc_scale = 2.841
gyro_scale = 0.2084
omega = 0
dt = 0.02
direction = 0

Ktau=0.304766706036738
B=0.000726962269165
Ar=0.007776695904018
J=0.001502739083882
Ra=6.832749059810827
Kb=0.459965726538748
La=0.00494
U=8



gyroSensor          = ev3dev.ev3.Sensor(address="in2")
accelSensor         = ev3dev.ev3.Sensor(address="in3")

leftWheelA           = ev3dev.ev3.LargeMotor('outA')
leftWheelB           = ev3dev.ev3.LargeMotor('outB')
rightWheelC          = ev3dev.ev3.LargeMotor('outC')
rightWheelD          = ev3dev.ev3.LargeMotor('outD')


motorEncoderStart   = rightWheelC.position 
loop = True

#timewrite  = open("time.txt", "w")

while loop:
    t1 = time.time()

    gyro = gyroSensor.value()                           #reading gyro value
    accelraw =  4*accelSensor.value(n=2) + accelSensor.value(n=5)    #reading accelerometer value
                                                                    #(our accelerometer is mounted so that we want to use its z-value)


    if accelraw > 600:              #defining values between 211-360 degrees as -149-0 degrees
        accelraw = accelraw - 1023
    

    accel = accelraw/acc_scale      #scaling accelerometers values to degrees
   # print(accel, " ", accelSensor.value(n=2)/(pow(10,accelSensor.decimals)))

    angularSpeed = (gyro - gyro_offset) * gyro_scale    #scaling gyro and removing offset

    angle = (0.95)*(angle- angularSpeed*dt) + (0.05)*(accel-4.2)   # Komplementär filter. Notera viktningen

  

    motorEncoderStop = rightWheelC.position 
    wheelSpeed = rightWheelC.speed


    wheelangle = motorEncoderStart - motorEncoderStop

    u = (0.0189  *wheelangle  -0.0392 *wheelSpeed -7.3837  *angle + 0.9384*angularSpeed)
   

    motorinput = u
 
    if motorinput > 90:
            motorinput = 90

    if motorinput < -90:
            motorinput = -90

    leftWheelA.run_direct(duty_cycle_sp = motorinput)
    leftWheelB.run_direct(duty_cycle_sp = motorinput)
    rightWheelC.run_direct(duty_cycle_sp = motorinput)
    rightWheelD.run_direct(duty_cycle_sp = motorinput)

   
    t2 = time.time()
    tdiff = t2-t1
  
    if tdiff < dt:
        time.sleep(dt - tdiff)

    #timewrite.write(str(tdiff))
    #timewrite.write('\n')
    #print(tdiff)
