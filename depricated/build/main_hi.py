#!/usr/bin/env python3

import time
from time import sleep
import ev3dev.ev3
import math

tau = 0
angle = 0
gyro_offset = 2971
acc_scale = 2.841*2
gyro_scale = 0.2084

dt = 0.02

ang=0


gyroSensor          = ev3dev.ev3.Sensor(address="in2")
accelSensor         = ev3dev.ev3.Sensor(address="in3")

leftWheelA           = ev3dev.ev3.LargeMotor('outA')

rightWheelD          = ev3dev.ev3.LargeMotor('outD')


motorEncoderStart   = rightWheelD.position
loop = True

tdiff=0
#gyrowrite  = open("gyro.txt", "w")
#accelwrite  = open("accel.txt", "w")
#anglewrite  = open("angle.txt", "w")
#angularspeedwrite  = open("angularspeed.txt", "w")
#timewrite  = open("time.txt", "w")
ts = time.time()
while loop:


    gyro = gyroSensor.value()                           #reading gyro value
    accelraw =  4*accelSensor.value(n=2) + accelSensor.value(n=5)    #reading accelerometer value
                                                                    #(our accelerometer is mounted so that we want to use its z-value)





    if accelraw > 600:              #defining values between 211-360 degrees as -149-0 degrees
        accelraw = accelraw - 1023


    accel = accelraw/acc_scale      #scaling accelerometers values to degrees


    angularSpeed = (gyro - gyro_offset) * gyro_scale    #scaling gyro and removing offset

    angle = (0.95)*(angle- angularSpeed*dt) + (0.05)*(accel-7.7)   # Komplementär filter. Notera viktningen




    motorEncoderStop = rightWheelD.position
    wheelSpeed = rightWheelD.speed


    wheelangle = motorEncoderStart - motorEncoderStop


    u = (0.0001*wheelangle  +0.0147 *wheelSpeed-180.3442  *angle +5.3554*angularSpeed)/360*2*3.14

    #u = ( -181.5713 *angle +5.0*angularSpeed)/360*2*3.14
    #u = ( -161.5713 *angle +4.5*angularSpeed)/360*2*3.14



    motorinput = u*10
    #ang=ang- angularSpeed*dt
    #gyrowrite.write(str(gyro))
    #gyrowrite.write('\n')
    #accelwrite.write(str(accel))
    #accelwrite.write('\n')
    #anglewrite.write(str(angle))
    #anglewrite.write('\n')
    #angularspeedwrite.write(str(ang))
    #angularspeedwrite.write('\n')
    #a=angularSpeed*dt
    #print(u, " ", motorinput," ",wheelangle, " ",wheelSpeed, " ", angle," ", angularSpeed)
    #print(angularSpeed, " ", motorinput)
    if motorinput > 90:
            motorinput = 90

    if motorinput < -90:
            motorinput = -90

    leftWheelA.run_direct(duty_cycle_sp = motorinput)
    rightWheelD.run_direct(duty_cycle_sp = motorinput)


    if angle > 20:
        leftWheelA.stop(stop_action="coast")
        rightWheelD.stop(stop_action="coast")
        loop=False



    tn = time.time()
    tdiff = tn-ts

    if tdiff < dt:
        time.sleep(dt - tdiff)
    ts=time.time()
    #timewrite.write(str(tdiff))
    #timewrite.write('\n')
    #print(tdiff)
