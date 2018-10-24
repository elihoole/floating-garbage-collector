import RPi.GPIO as GPIO
import time
import numpy as np
inPINS = [2,3,4,14,15,18,17,27,22,23]
smoothingWindowLength= 6

'''copied from operations.py'''

from nanpy import Servo
from time import sleep
import getch
from nanpy import (ArduinoApi, SerialManager)

timeout = None

#initialise Thruster and directionalServo
Thruster = Servo(8)
directionalServo = Servo(7)

Thruster.write(1500)
directionalServo.write(83)
time.sleep(3)

#initialise lifter
connection = SerialManager()
a = ArduinoApi(connection = connection)
a.pinMode(5,a.OUTPUT)
a.pinMode(6,a.OUTPUT)

a.digitalWrite(5, a.HIGH)
a.digitalWrite(6, a.HIGH)

def drive_thruster(jostick_out_vol_1) :
    Thruster.write(joystick_out_vol_1)

def drive_directional_servo(joystick_out_vol_2) :
    directionalServo.write(joystick_out_vol_2)

def drive_lifter_arm(joystick_out_vol_3) :
    if joystick_out_vol_3 > 0 :
        a.digitalWrite(5, a.LOW)
        a.digitalWrite(6, a.LOW)
    elif joystick_out_vol_3 < 0 :
        a.digitalWrite(5, a.HIGH)
        a.digitalWrite(6, a.HIGH)

def getTimex():
    return time.time()

GPIO.setmode(GPIO.BCM)
GPIO.setup(inPINS, GPIO.IN)
upTimes = [[0] for i in range(len(inPINS))]
downTimes = [[0] for i in range(len(inPINS))]
deltaTimes = [[0] for i in range(len(inPINS))]

def my_callback1(channel):
    i = inPINS.index(channel)
    v = GPIO.input(inPINS[i])
  #GPIO.output(outPINS[0], v) # mirror input state to output state directly (forward servo value only) - don't set PWM then for this pin
    if (v==0):
        downTimes[i].append(getTimex())
    if len(downTimes[i])>smoothingWindowLength:
        del downTimes[i][0]
    else:
        upTimes[i].append(getTimex())
    if len(upTimes[i])>smoothingWindowLength:
        del upTimes[i][0]
    deltaTimes[i].append( (downTimes[i][-1]-upTimes[i][-2])/(upTimes[i][-1]-downTimes[i][-1]) )
    if len(deltaTimes[i])>smoothingWindowLength:
        del deltaTimes[i][0]

GPIO.add_event_detect(inPINS[0], GPIO.BOTH, callback=my_callback1)
GPIO.add_event_detect(inPINS[1], GPIO.BOTH, callback=my_callback1)
GPIO.add_event_detect(inPINS[6], GPIO.BOTH, callback=my_callback1)

def translate(value, leftmin, leftmax, rightmin, rightmax):
    leftspan=leftmax-leftmin
    rightspan=rightmax-rightmin
    valuescaled=float(value-leftmin)/float(leftspan)
    return rightmin+(valuescaled*rightspan)


try:
  while True:
        ovl0 = deltaTimes[0][-smoothingWindowLength:] # output 1ST pin PWM
        ovl1 = deltaTimes[1][-smoothingWindowLength:] # output 2ND pin PWM
        ovl2 = deltaTimes[6][-smoothingWindowLength:] # output 3rd pin PWM

        #print('*****', type(ovl1), ovl1)
        ov0 = sorted(ovl0)[len(ovl0)//2] #ov = np.mean(ovl)
        ov1 = sorted(ovl1)[len(ovl1)//2] #ov = np.mean(ovl)
        ov2 = sorted(ovl2)[len(ovl2)//2] #ov = np.mean(ovl)

        ov0 = translate(ov0 , 0.04798502990289330 , 0.10066968601432865 ,1100 , 1900 )
        ov0 = int(ov0)

        ov1 = translate(ov1 , 0.05198502990289330 , 0.11066968601432865 ,30 , 285 )
        ov1 = int(ov1)

        ov2 = translate(ov2 , 0.04798502990289330 , 0.10066968601432865 ,-10 , 10 )
        ov2 = int(ov2)


        print("Thruster : ", ov0, "| Servo : ", ov1, "| Mode : ", ov2 )

        drive_thruster(ov0)
        drive_directional_servo(ov1)
        drive_lifter_arm(ov2)
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
