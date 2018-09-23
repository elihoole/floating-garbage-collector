import serial               #import serial pacakge
from time import sleep
import webbrowser           #import package for opening link in browser
import sys                  #import system package
import math
import cv2
import sys
sys.path.append('/home/pi/project/code')
from i2clibraries import i2c_hmc5883l
from nanpy import Servo
from nanpy import (ArduinoApi, SerialManager)
import gmplot

timeout = None 

Thruster = Servo(8)
directionalServo = Servo(7)

connection = SerialManager()
a = ArduinoApi(connection = connection)
a.pinMode(5, a.OUTPUT)
a.pinMode(6, a.OUTPUT)

a.digitalWrite(5, a.LOW)
a.digitalWrite(6, a.LOW)

def GPS_Info():
    global NMEA_buff
    global lat_in_degrees
    global long_in_degrees
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
    nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
    nmea_longitude = NMEA_buff[3]               #extract longitude from GPGGA string
    
    #print("NMEA Time: ", nmea_time,'\n')
    #print ("NMEA Latitude:", nmea_latitude,"NMEA Longitude:", nmea_longitude)

    if (len(nmea_latitude) > 0 and len(nmea_longitude) > 0):
        lat = float(nmea_latitude)                  #convert string into float for calculation
        longi = float(nmea_longitude)               #convertr string into float for calculation
        
        lat_in_degrees = convert_to_degrees(lat)    #get latitude in degree decimal format
        long_in_degrees = convert_to_degrees(longi) #get longitude in degree decimal format
    
#convert raw NMEA string into degree decimal format   
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.8f" %(position)
    return position
    
gpgga_info = "$GPGGA,"
ser = serial.Serial ("/dev/ttyAMA0")              #Open port with baud rate
GPGGA_buffer = 0
NMEA_buff = 0
lat_in_degrees = 0
long_in_degrees = 0

#PointB_lat = float(input("Enter latitude of destination : "))
#PointB_long = float(input("Enter longitude of destination : "))

#PointB_lat = 7.30023055
#PointB_long = 81.8649878

PointB_lat = 7.30052413
PointB_long = 81.86492753

PointB = (PointB_lat, PointB_long)
sleep(0.2)

lat_map = []
long_map = []

try:
    while True:
        lat_map.append(float(lat_in_degrees)) 
        long_map.append(float(long_in_degrees))
        
        received_data = (str)(ser.readline())                   #read NMEA string received
        GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string                 
        if (GPGGA_data_available>0):
            GPGGA_buffer = received_data.split("$GPGGA,",1)[1]  #store data coming after "$GPGGA," string 
            NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
            
            GPS_Info()                                          #get time, latitude, longitude
 
            print("Current GPS Latitude : ", lat_in_degrees," Longitude : ", long_in_degrees)
            print ("Destination GPS Latitude :  ", PointB_lat, " Longitude : ", PointB_long)
            PointA_lat = float(lat_in_degrees)
            PointA_long = float(long_in_degrees)

            PointA = (PointA_lat, PointA_long)

            M = math.cos(math.radians(PointB[0]))*math.sin(math.radians(PointB[1]) - math.radians(PointA[1]))
            N = math.cos(math.radians(PointA[0]))*math.sin(math.radians(PointB[0]))- math.sin(math.radians(PointA[0]))*math.cos(math.radians(PointB[0]))*math.cos(math.radians(PointB[1])-math.radians(PointA[1]))

            BearingAngle = math.atan2(M,N)

            hmc5883l = i2c_hmc5883l.i2c_hmc5883l(1)
            hmc5883l.setContinuousMode()
            hmc5883l.setDeclination(0,6)
            HeadingAngle = hmc5883l.getHeading()


            if (HeadingAngle[0] > 180):
                HeadingAngleX = -(360 - HeadingAngle[0])
            else:
                HeadingAngleX = HeadingAngle[0]

            Difference_Angle = math.degrees(BearingAngle) - HeadingAngleX

            print(Difference_Angle, 'xx')

            if (Difference_Angle > 180):
                Difference_Angle = Difference_Angle - 360

            if (Difference_Angle < -180):
                Difference_Angle = Difference_Angle + 360

            if (Difference_Angle < 5 and Difference_Angle > -5):
                Difference_Angle = Difference_Angle*3
            
            #print('Heading angle : ', HeadingAngleX)
            print('Bearing Angle = ', math.degrees(BearingAngle))
            print('Difference Angle = ', Difference_Angle, '\n')

            lon1, lat1, lon2, lat2 = map(math.radians, [PointA_long, PointA_lat, PointB_long, PointB_lat])

            # haversine formula 
            dlon = lon2 - lon1 
            dlat = lat2 - lat1 
            a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
            c = 2 * math.asin(math.sqrt(a)) 
            r = 6371 # Radius of earth in kilometers. Use 3956 for miles
            Range = (r*c)*1000
            Range1 = "%.2f" %(Range)
            print ('Distance for destination is : ' , Range1, ' meters')

            if (int(Range) > 3):
                speed = 1700
                if (int  (Difference_Angle) == 0):
                    turn = 85
                    directionalServo.write(int (turn))
                    Thruster.write(speed)
                    print( 'Turn = ', int(turn), '\nSpeed = ', speed)
                    sleep(0.2)    
                else:
                    turn = 85 + Difference_Angle/4
                    directionalServo.write(int (turn))
                    Thruster.write(speed)
                    print('Turn = ', int(turn), '\nSpeed = ', speed)
                    sleep(0.2)

            else:
                directionalServo.write(83)
                Thruster.write(1500)
            
            print("------------------------------------------------------------------------\n")

            ser.flush()
 
except KeyboardInterrupt:
    directionalServo.write(83)
    Thruster.write(1500)
    
    lat_map1 = []
    long_map1 = []
    k = 20
    while (k <= len(lat_map)):
        lat_map1.append(lat_map[k])
        long_map1.append(long_map[k])
        k = k + 100   
    gmap = gmplot.GoogleMapPlotter(lat_map1[0],long_map1[0], 22)
    gmap.plot(lat_map1, long_map1, 'black', edge_width=5)
    gmap.scatter(lat_map1, long_map1, '#FF6666', size=0.3, edge_width=0.2, marker=False)
    gmap.scatter(lat_map1, long_map1, 'red', marker=True)
    gmap.draw('Trajectory_map.html')

    sys.exit(0)
