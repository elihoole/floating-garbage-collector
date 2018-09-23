from nanpy import Servo
from time import sleep
import getch
from nanpy import (ArduinoApi, SerialManager)

timeout = None


servo1 = Servo(8)
servo2 = Servo(7)

turn = 83
servo2.write(turn)

connection = SerialManager()
a = ArduinoApi(connection = connection)
a.pinMode(5,a.OUTPUT)
a.pinMode(6,a.OUTPUT)

a.digitalWrite(5, a.HIGH)
a.digitalWrite(6, a.HIGH)

while 1:
    x = input("Enter Command : ")
    if x == 'x':
        stopsignal = 1500
        turn = 85
        servo1.write(stopsignal)
        servo2.write(turn)

    elif x == 'w':
        fwsignal = 1750
        turn = 85
        servo1.write(fwsignal)
        servo2.write(turn)
        
    elif x == 'd':
        fwsignal =1600
        turn = turn + 15
        servo1.write(fwsignal)
        servo2.write(turn)
        print(turn - 80,' degrees from centre \n')
    elif x == 'a':
        fwsignal = 1600
        turn = turn - 15
        servo1.write(fwsignal)
        servo2.write(turn)
        print(80 - turn,' degrees from centre \n')
    elif x == 's':
        rwsignal = 1400
        turn = 83
        servo1.write(rwsignal)
        servo2.write(turn)
    elif x == 'u':
        a.digitalWrite(5, a.LOW)
        a.digitalWrite(6, a.LOW)
    elif x == 'i':
        a.digitalWrite(5, a.HIGH)
        a.digitalWrite(6, a.HIGH)
    else:
        print("use the following keys instead \n w = forward \n s = reverse \n a = left \n d = right \n u = Lifter UP \n i = Lifter DOWN \n\n")


        
    
