#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
	
class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.camera_callback)
	#self.vel_pubDrone = rospy.Publisher ("/uav1/cmd_vel", Twist, queue_size=1)
	self.a = 0
	self.b = 0
	self.msg_cube = rospy.Publisher("/pose_cube",Point, queue_size=1)

    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

	# Se recorta la imagen para disminuir el tiempo de procesamiento de imagen
	height, width, channels = cv_image.shape
	#descentre = 160
	#rows_to_watch = 60
	#crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
	
	#Convertimos la imagen a un formato de opencv
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

	# We convert the imagen to white and color using a mask
	upper_red = np.array([20, 255,255])
	lower_red = np.array([0, 180, 180])
	mask_r = cv2.inRange(hsv, lower_red, upper_red)

	upper_green = np.array([80, 255,255])
	lower_green = np.array([60, 180, 180])
	mask_g = cv2.inRange(hsv, lower_green, upper_green)

	upper_blue = np.array([130, 255,255])
	lower_blue = np.array([100, 180, 180])
	mask_b = cv2.inRange(hsv, lower_blue, upper_blue)

	upper_yellow = np.array([50, 255,255])
	lower_yellow = np.array([20, 180, 180])
	mask_y = cv2.inRange(hsv, lower_yellow, upper_yellow)

	#Computing the centroid of the blob
	mr = cv2.moments(mask_r, False)
	try:
	    cxr, cyr = mr['m10']/mr['m00'], mr['m01']/mr['m00']
	except ZeroDivisionError:
	    cyr, cxr = height/2, width/2

	mg = cv2.moments(mask_g, False)
	try:
	    cxg, cyg = mg['m10']/mg['m00'], mg['m01']/mg['m00']
	except ZeroDivisionError:
	    cyg, cxg = height/2, width/2

	mb = cv2.moments(mask_b, False)
	try:
	    cxb, cyb = mb['m10']/mb['m00'], mb['m01']/mb['m00']
	except ZeroDivisionError:
	    cyb, cxb = height/2, width/2

	my = cv2.moments(mask_y, False)
	try:
	    cxy, cyy = my['m10']/my['m00'], my['m01']/my['m00']
	except ZeroDivisionError:
	    cyy, cxy = height/2, width/2

#	print('--cxr--',cxr,'--cyr--', cyr)
	print('--cxg--',cxg,'--cyg--', cyg)
#	print('--cxb--',cxb,'--cyb--', cyb)
#	print('--cxy--',cxy,'--cyy--', cyy)

        #cv2.imshow("Image window", np.hstack([crop_img,extraction]))
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)
	

	if cyg < 400:
	    cmy = (399.5 -float(cyg)) / 900
	    #print (cmy)
	    self.a = (0.70 + cmy)
#	    numero = a
	    texto = "Valor de x= " 
	    print (texto+" "+str(self.a))
    

	    if cxg < 400:
	        cmx = (399.5 - float(cxg)) / 900
	     #   print (cmx)
	        self.b = (0 + cmx)
#	        numero = b
	        texto = "Valor de y= " 
	        print (texto+" "+str(self.b))
	    else:
	        cmx = (399.5 - float(cxg)) / 900
	      #  print (cmx)
	        self.b = (0 + cmx)
#	        numero = b
	        texto = "Valor de y= " 
	        print (texto+" "+str(self.b))
	else:
	    cmy = (399.5 -float(cyg)) / 900
	   # print (cmy)
	    self.a = (0.70 + cmy)
#	    numero = a
	    texto = "Valor de x= " 
	    print (texto+" "+str(self.a))

	    if cxg < 400:
	        cmx = (399.5 - float(cxg)) / 900
	        #print (cmx)
	        self.b = (0 + cmx)
#	        numero = b
	        texto = "Valor de y= " 
	        print (texto+" "+str(self.b))
	    else:
	        cmx = (399.5 - float(cxg)) / 900
	       # print (cmx)
	        self.b = (0 + cmx)
#	        numero = b
	        texto = "Valor de y= " 
	        print (texto+" "+str(self.b))

	PoseCubeGreen = Point ()
	PoseCubeGreen.x = self.a
	PoseCubeGreen.y = self.b 
	PoseCubeGreen.z = -0.14000000000000
	
	self.msg_cube.publish(PoseCubeGreen)

def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
