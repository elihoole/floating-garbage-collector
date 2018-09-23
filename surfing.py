import cv2
#import matplotlib
import numpy as np
import sys
cap = cv2.VideoCapture('My_Movie_5.mp4')
cap.set(3,320)                             #set width 
cap.set(4,240)								#set height 

''' this is python script implements the detector part of SURF'''

while True:
	try:     
		ret, img = cap.read()
		if img == None:
			raise Exception("image could not be loaded! check file path")
		height, width, channels = img.shape
		#print(height, width, channels)
		
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)      	
		
		#set up SURF with hessian thresh hold 4000
		surf = cv2.xfeatures2d.SURF_create(4000)
		surf.setUpright(True)
		key_points = surf.detect(gray, None)
		
		Points_for_Circle_Drawing = [] 
		
		for ix in range (len(key_points)):
			pts_x, pts_y = (key_points[ix].pt)
			pts_x, pts_y = int(pts_x), int(pts_y) 

			Points_for_Circle_Drawing.append((pts_x, pts_y))
			cv2.circle(img, Points_for_Circle_Drawing[-1], 5, (0,0,255), -1)

		cv2.imshow('detected keypoins in red', img)
	
		#quit and close everything with keyboard input 'q'
		k = cv2.waitKey(1) & 0xFF
		if k == ord('q'):
			cv2.destroyAllWindows()
			cap.release()
			quit()

	#quit and close everything with keyboard input 'ctrl + c'
	except KeyboardInterrupt:
		sys.exit(0)
		cv2.destroyAllWindows()
		cap.release()


