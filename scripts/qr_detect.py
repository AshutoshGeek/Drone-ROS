#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from pyzbar.pyzbar import decode

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test') #Initialise rosnode 
		self.img = np.empty([])
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		# This will contain your image frame from camera
		self.bridge = CvBridge()
		self.target = [0.0, 0.0, 0.0]
		self.targeti = [0.0, 0.0, 0.0]


	# Callback function of camera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		except CvBridgeError as e:
			print(e)
			return

		cv2.imshow("Image window", self.img)
		cv2.waitKey(3)
		self.img_decoded = decode(self.img)
		#rospy.loginfo(type(img_decoded[0][0]))

		self.targeti = self.img_decoded[0][0].split(",")
		self.target[0] = float(self.targeti[0])
		self.target[1] = float(self.targeti[1])
		self.target[2] = float(self.targeti[2])
		
		rospy.loginfo(self.target)
		
if __name__ == '__main__':
	image_proc_obj = image_proc()
	rospy.spin()