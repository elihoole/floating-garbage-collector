import cv2
import time
import numpy as np
from nanpy import Servo
from nanpy import (ArduinoApi, SerialManager)
import serial               #import serial pacakge
from time import sleep
import webbrowser           #import package for opening link in browser
import sys                  #import system package
sys.path.append('/home/pi/project/code')
from i2clibraries import i2c_hmc5883l
import math
import gmplot
import webbrowser

timeout = None 

Thruster = Servo(8)
directionalServo = Servo(7)


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

print('Initialising . . .')
StartPoint = []
for count in range (30):
    received_data = (str)(ser.readline())                   #read NMEA string received
    GPGGA_data_available = received_data.find(gpgga_info)  #check for NMEA GPGGA string
    if (GPGGA_data_available>0):
        GPGGA_buffer = received_data.split("$GPGGA,",1)[1]  #store data coming after "$GPGGA," string 
        NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
                
        GPS_Info()                                          #get time, latitude, longitude
     
        #print("Current GPS Latitude : ", lat_in_degrees," Longitude : ", long_in_degrees)
        #print ("Destination GPS Latitude :  ", PointB_lat, " Longitude : ", PointB_long)
        Start_lat = float(lat_in_degrees)
        Start_long = float(long_in_degrees)

StartPoint = (Start_lat, Start_long)
print('Starting location is :' ,StartPoint)

GPS_points =[]

GPS_points.append(StartPoint)
GPS_points.append((float(7.300223932), float(81.86496399)))
GPS_points.append((float(7.30052679), float(81.8648953457)))
GPS_points.append((float(7.300466575), float(81.86465696)))
GPS_points.append((float(7.3002723618), float(81.86465696)))

#for  k in range (4):
    #Point_Latitude = float(input("Enter latitude of point : "))
    #Point_Longitude = float(input("Enter latitude of point : "))
    #GPS_points.append((float(Point_Latitude), float(Point_Longitude)))

#Find midpoint between First Point  and Final Point

lat_first, _ = GPS_points[1]
_, long_first =GPS_points[1]

lat_last, _ = GPS_points[4]
_, long_last = GPS_points[4]

lat_first, long_first, lat_last, long_last = map(math.radians, [lat_first, long_first, lat_last, long_last])

bx = math.cos(lat_last)*math.cos(long_last - long_first)
by = math.cos(lat_last)*math.sin(long_last - long_first)

lat_mid = math.atan2(math.sin(lat_first)+math.sin(lat_last), \
                     math.sqrt((math.cos(lat_first)+bx)*(math.cos(lat_first)+bx) + by**2))
long_mid = long_first + math.atan2(by, math.cos(lat_first) + bx)


ThirdPoint = (round (math.degrees(lat_mid),7), round (math.degrees(long_mid),7))

GPS_points.insert(3, ThirdPoint)
GPS_points.insert(6, GPS_points[1])
GPS_points.insert(7, StartPoint)

GPS_lat = []
GPS_long = []

for GPSloop in range(len(GPS_points)):
    GPS_lat_x, _ = GPS_points[GPSloop]
    _, GPS_long_y = GPS_points[GPSloop]
    GPS_lat.append(GPS_lat_x)
    GPS_long.append(GPS_long_y)

lat_map = []
long_map = []

################################################################################################################

kernel1 = np.ones((5,5),np.uint8)

cap = cv2.VideoCapture(0)                  #read from USB connected camera
cap.set(3,320)                             #set width 
cap.set(4,240)                             #set height 



connection = SerialManager()
API = ArduinoApi(connection = connection)
API.pinMode(5, API.OUTPUT)
API.pinMode(6, API.OUTPUT)

API.digitalWrite(5, API.HIGH)
API.digitalWrite(6, API.HIGH)

hmc5883l = i2c_hmc5883l.i2c_hmc5883l(1)
hmc5883l.setContinuousMode()
hmc5883l.setDeclination(0,6)

lower_red = np.array([0,100,100]) #0 100 100                                #set threshold for Red
upper_red = np.array([10,255,255]) #10 255 255    

print('------------------------------------------------------------------')
try:
    j = 1
    #print("Always GPS")
    while (j < 8):


            ret, img = cap.read()                                                       #read continuous frames from camera 
            hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                               #convert to HSV    
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

            #print('\n', horizontalPD, 'Horizontal PD \n')

            if(len(verticalPD) > 0):                        #robotic action definitions 
                print("Object detected")
                if (verticalPD[0] > 25):
                    API.digitalWrite(5, API.HIGH)
                    API.digitalWrite(6, API.HIGH)
                    
                    if (len(horizontalPD) > 0):
                        print('Navigating towards object')
                        speed = 1650
                        turn = 85
                        if (horizontalPD[0] > 0):
                            turn = 85 + horizontalPD[0]/4
                            directionalServo.write(turn)
                            Thruster.write(speed)

                        elif (horizontalPD[0] < 0 ):
                            turn = 85 + horizontalPD[0]/4
                            directionalServo.write(turn)
                            Thruster.write(speed)

                        elif (horizontalPD[0] == 0):
                            turn = 85
                            speed = 1650
                            directionalServo.write(turn)
                            Thruster.write(speed)
                        
                elif (verticalPD[0]< 25):
                    turn = 83
                    speed = 1500
                    directionalServo.write(turn)
                    Thruster.write(speed)
                    API.digitalWrite(5, API.LOW)
                    API.digitalWrite(6, API.LOW)

                    print("Picking up object")
                print('---------------------************************---------------------')

                ser.flush()

            else:
                API.digitalWrite(5, API.HIGH)
                API.digitalWrite(6, API.HIGH)

                PointB_lat, _ = GPS_points[j]
                _, PointB_long = GPS_points[j]
                PointB = (PointB_lat, PointB_long)

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

                    #hmc5883l = i2c_hmc5883l.i2c_hmc5883l(1)
                    #hmc5883l.setContinuousMode()
                    #hmc5883l.setDeclination(0,6)
                    HeadingAngle = hmc5883l.getHeading()
                        

                    if (HeadingAngle[0] > 180):
                            HeadingAngleX = -(360 - HeadingAngle[0])
                    else:
                        HeadingAngleX = HeadingAngle[0]

                    Difference_Angle = math.degrees(BearingAngle) - HeadingAngleX

                    if (Difference_Angle > 180):
                        Difference_Angle = Difference_Angle - 360
                    elif (Difference_Angle < -180):
                        Difference_Angle = Difference_Angle + 360

                    
                                
                    #print('Heading angle : ', HeadingAngleX)
                    #print('Bearing Angle = ', math.degrees(BearingAngle))
                    #print('Difference Angle = ', Difference_Angle, '\n')


                    # haversine formula
                    lon1, lat1, lon2, lat2 = map(math.radians, [PointA_long, PointA_lat, PointB_long, PointB_lat])
                    dlon = lon2 - lon1 
                    dlat = lat2 - lat1 
                    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
                    c = 2 * math.asin(math.sqrt(a)) 
                    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
                    Range = (r*c)*1000
                    Range1 = "%.2f" %(Range)
                    print ('Distance for destination is : ' , Range1, ' meters')
                    
                    if (int(Range) < 3):
                        j = j + 1
                    else:    
                        speed = 1750
                        turn = 85
                        if (int  (Difference_Angle) == 0):
                            turn = 85
                            if (turn > 145 or turn < 25):
                                turn = 85 
                            directionalServo.write(int (turn))
                            
                            Thruster.write(speed)
                            #print( 'Turn = ', int(turn), '\nSpeed = ', speed)
                                   
                        else:
                            turn = 85 + Difference_Angle/3
                            
                            directionalServo.write(int (turn))
                            Thruster.write(speed)
                            #print('Turn = ', int(turn), '\nSpeed = ', speed)
                    ser.flush()
                                
                    print('---------------------************************---------------------')
            cv2.imshow('Detection in blue', img) 
            #cv2.imshow('capture',mask1)
        
            k =cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                directionalServo.write(85)
                Thruster.write(1500)
                API.digitalWrite(5, API.HIGH)
                API.digitalWrite(6, API.HIGH)
                break
            ser.flush()
            if (j == 8):
               directionalServo.write(83)
               Thruster.write(1500)
               lat_map1 = []
               long_map1 = []
               k = 20
               while (k <= len(lat_map)):
                   lat_map1.append(lat_map[k])
                   long_map1.append(long_map[k])
                   k = k + 50
               gmap = gmplot.GoogleMapPlotter(lat_map1[0],long_map1[0], 20)
               gmap.plot(lat_map, long_map, 'black', edge_width=5)
               gmap.scatter(lat_map1, long_map1, '#FF6666', size=0.3, edge_width=0.2, marker=False)
               #gmap.scatter(lat_map, long_map, '#FF6666', size=0.001, edge_width=0.001, marker=False)
               gmap.scatter(lat_map1, long_map1, '#90EE90', marker=True)
               gmap.scatter(GPS_lat, GPS_long, 'red', marker=True)
               gmap.draw('Vision_GPS_Trajectory_map.html')
               map_link = '/home/pi/Desktop/AFGC/Vision_GPS_Trajectory_map.html'
               print ("*** At starting point - TERMINATE ***")
               cv2.destroyAllWindows()
               webbrowser.open(map_link)
               cap.release()
               sys.exit(0)

except KeyboardInterrupt:
    print("\n\n------------- Keyboard Interrupt - TERMINATING the program -------------\n")
    directionalServo.write(83)
    Thruster.write(1500)
    lat_map1 = []
    long_map1 = []
    k = 20
    while (k <= len(lat_map)):
        lat_map1.append(lat_map[k])
        long_map1.append(long_map[k])
        k = k + 50
    gmap = gmplot.GoogleMapPlotter(lat_map1[0],long_map1[0], 20)
    gmap.plot(lat_map, long_map, 'black', edge_width=5)
    gmap.scatter(lat_map1, long_map1, '#FF6666', size=0.3, edge_width=0.2, marker=False)
    #gmap.scatter(lat_map, long_map, '#FF6666', size=0.001, edge_width=0.001, marker=False)
    gmap.scatter(lat_map1, long_map1, '#90EE90', marker=True)
    gmap.scatter(GPS_lat, GPS_long, 'red', marker=True)
    gmap.draw('Vision_GPS_Trajectory_map.html')
    map_link = '/home/pi/Desktop/AFGC/Vision_GPS_Trajectory_map.html'
    cv2.destroyAllWindows()
    cap.release()
    webbrowser.open(map_link)
    sys.exit(0)
cv2.destroyAllWindows()
cap.release()
