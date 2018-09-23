import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
from nanpy import Servo
from nanpy import (ArduinoApi, SerialManager)

turn = 83                                  #initialise directionalServo
directionServo = Servo(7)
directionServo.write(turn)

speed = 1500                               #initialise Thruster                             
Thruster = Servo(8)
Thruster.write(speed)


connection = SerialManager()               #initialise Serial connection to Aruduino
a = ArduinoApi(connection = connection)
a.pinMode(5,a.OUTPUT)
a.pinMode(6,a.OUTPUT)

a.digitalWrite(5, a.HIGH)                  #set lifter down
a.digitalWrite(6, a.HIGH)

kernel1 = np.ones((1,1),np.uint8)

cap = cv2.VideoCapture(0)                  #read from USB connected camera
cap.set(3,320)                             #set width 
cap.set(4,240)                             #set height 



while(True):

    ret, img = cap.read()                                                       #read continuous frames from camera
    hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                               #convert to HSV


    lower_red = np.array([175,100,100]) #0 100 100                                #set threshold for Red
    upper_red = np.array([180,255,255]) #10 255 255    

    
    mask1 = cv2.inRange(hsvimg, lower_red, upper_red)                             #mask all colors except red     
    

    mask2 = cv2.erode(mask1,kernel1,iterations = 1)                                #erode regions too small (noise)
    mask2 = cv2.dilate(mask2, kernel1, iterations = 2)                             #dialate disconitnuous regions

    mask2 = cv2.blur(mask2,(4,4))                                                  #smoothen edges
    
    _, contours, _= cv2.findContours(mask2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  #find contours

    height, width = img.shape[:2]
    centres = []
    horizontalPD = []
    verticalPD = []

    for i in range(len(contours)):
      moments = cv2.moments(contours[i])                                                              
      if (moments['m00']>0):
          centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))   # find centres of contours and store in centres array
          horizontalPD.append(int (moments['m10']/moments['m00'] - width/2))                         # find horizontal pixel difference and store in horizontalPD array
          verticalPD.append(int(height - moments['m01']/moments['m00']))                             # find vertical pixel difference and store in verticalPD array
    
          cv2.circle(img, centres[-1], 5, (255, 0, 0), -1)                                            # draw blue circles at centres of contours the actual frame
          
    if(len(verticalPD) > 0):                                                                         #robotic action definitions 
        if (verticalPD[0] > 25):
            a.digitalWrite(5, a.HIGH)
            a.digitalWrite(6, a.HIGH)
            if (len(horizontalPD) > 0):
                speed = 1590
                if (horizontalPD[0] > 0):
                    turn = 83 + horizontalPD[0]/4
                    directionServo.write(turn)
                    Thruster.write(speed)
                    time.sleep(0.2)

                elif (horizontalPD[0] < 0 ):
                    turn = 83 + horizontalPD[0]/4
                    directionServo.write(turn)
                    Thruster.write(speed)
                    time.sleep(0.2)

                elif (horizontalPD[0] == 0):
                    turn = 83
                    speed = 1590
                    directionServo.write(turn)
                    Thruster.write(speed)
            
        elif (verticalPD[0]< 25):
            turn = 83
            speed = 1500
            directionServo.write(turn)
            Thruster.write(speed)
            time.sleep(1)
            a.digitalWrite(5, a.LOW)
            a.digitalWrite(6, a.LOW)
            
    else:
        Thruster.write(1500)
        directionServo.write(83)
        a.digitalWrite(5, a.HIGH)
        a.digitalWrite(6, a.HIGH)
      
    cv2.imshow('original', img) 
    cv2.imshow('capture',mask1)
    
    k =cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        directionServo.write(83)
        Thruster.write(1500)
        a.digitalWrite(5, a.HIGH)
        a.digitalWrite(6, a.HIGH)
        
        break


cv2.destroyAllWindows()
cap.release()
