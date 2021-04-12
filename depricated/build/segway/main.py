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
omega = 0
dt = 0.03
direction = 0

Ktau=0.304766706036738
B=0.000726962269165
Ar=0.007776695904018
J=0.001502739083882
Ra=6.832749059810827
Kb=0.459965726538748
La=0.00494
U=8

ang=0


gyroSensor          = ev3dev.ev3.Sensor(address="in2")
accelSensor         = ev3dev.ev3.Sensor(address="in3")

leftWheelA           = ev3dev.ev3.LargeMotor('outA')
leftWheelB           = ev3dev.ev3.LargeMotor('outB')
rightWheelC          = ev3dev.ev3.LargeMotor('outC')
rightWheelD          = ev3dev.ev3.LargeMotor('outD')


motorEncoderStart   = rightWheelC.position 
loop = True
motor=True
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
   # print(accel, " ", accelSensor.value(n=2)/(pow(10,accelSensor.decimals)))

    angularSpeed = (gyro - gyro_offset) * gyro_scale    #scaling gyro and removing offset

    angle = (0.93)*(angle- angularSpeed*dt) + (0.07)*(accel-2.4)   # Komplementär filter. Notera viktningen
    
    #print(*accelSensor.commands)


    motorEncoderStop = rightWheelC.position 
    wheelSpeed = rightWheelC.speed


    wheelangle = motorEncoderStart - motorEncoderStop
  

   # u = ( 0.0003  *wheelangle  -0.0095   *wheelSpeed  -32.2005    *angle + 1.0097*angularSpeed)
    #u = (0.0148  *wheelangle  -0.0308   *wheelSpeed -5.9077 *angle + 0.7750*angularSpeed)
    u = ( 0.0258  *wheelangle  +0.0592*wheelSpeed  -23.2323  *angle + 4.1598*angularSpeed)#/360*2*3.14


    motorinput = u#*10
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
    #print(u, " ", motorinput," ",tdiff)
    if motorinput > 90:
            motorinput = 90

    if motorinput < -90:
            motorinput = -90

    #leftWheelA.run_direct(duty_cycle_sp = motorinput)
    #leftWheelB.run_direct(duty_cycle_sp = motorinput)
    #rightWheelC.run_direct(duty_cycle_sp = motorinput)
    #rightWheelD.run_direct(duty_cycle_sp = motorinput)

    if motor:

        if motorinput > 90:
            motorinput = 90

        if motorinput < -90:
            motorinput = -90

        if motorinput > 0:
        

            if direction == 1:
                leftWheelB.run_direct(duty_cycle_sp = motorinput)
                rightWheelC.run_direct(duty_cycle_sp = motorinput)
                leftWheelA.stop(stop_action="coast")
                rightWheelD.stop(stop_action="coast")
                
              
            else:
                #leftWheelB.stop(stop_action="coast")
                #rightWheelC.stop(stop_action="coast")
                leftWheelB.run_direct(duty_cycle_sp = motorinput)
                rightWheelC.run_direct(duty_cycle_sp = motorinput)
                leftWheelA.stop(stop_action="coast")
                rightWheelD.stop(stop_action="coast")
                #leftWheelA.run_direct(duty_cycle_sp = 0)
                #rightWheelD.run_direct(duty_cycle_sp = 0)
                #leftWheelA.run_direct(speed_sp=-1)
                #rightWheelD.run_direct(speed_sp=-1)
            direction = 1
   
        if motorinput < 0:
        
            if direction == -1:
                leftWheelA.run_direct(duty_cycle_sp = motorinput)
                rightWheelD.run_direct(duty_cycle_sp = motorinput)
                leftWheelB.stop(stop_action="coast")
                rightWheelC.stop(stop_action="coast")

            else:
                #leftWheelA.stop(stop_action="coast")
                #rightWheelD.stop(stop_action="coast")
                leftWheelA.run_direct(duty_cycle_sp = motorinput)
                rightWheelD.run_direct(duty_cycle_sp = motorinput)
                leftWheelB.stop(stop_action="coast")
                rightWheelC.stop(stop_action="coast")
                #leftWheelB.run_direct(duty_cycle_sp = 0)
                #rightWheelC.run_direct(duty_cycle_sp = 0)
                #leftWheelB.run_direct(speed_sp=1)
                #rightWheelC.run_direct(speed_sp=1)
            direction = -1



    if angle > 20:
        leftWheelA.stop(stop_action="coast")
        leftWheelB.stop(stop_action="coast")
        rightWheelC.stop(stop_action="coast")
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
