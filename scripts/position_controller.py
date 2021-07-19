#!/usr/bin/env python3


# Importing the required libraries

from vitarana_drone.msg import *
from vitarana_drone.srv import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from std_msgs.msg import Float32, String
import rospy
import time
import tf

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib import pyplot as plt
import numpy as np
#from pyzbar.pyzbar import decode


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		#qr_detect
		rospy.init_node('position_controller') #Initialise rosnode 
		self.img = np.empty([])
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		# This will contain your image frame from camera
		self.bridge = CvBridge()
		logo_cascade = cv2.CascadeClassifier('data/cascade.xml')
		
		# Format for drone_command
		self.cmd_drone = edrone_cmd()
		self.cmd_drone.rcRoll = 1500
		self.cmd_drone.rcPitch = 1500
		self.cmd_drone.rcYaw = 1500
		self.cmd_drone.rcThrottle = 0

		self.cmd_gripper = Gripper()
		self.cmd_gripper.activate_gripper = False
		self.cmd_gripper.result = False

		# The latitude, longitude and altitude of the drone
		self.latitude = 0
		self.longitude = 0
		self.altitude = 0

		self.count = 0
		# The coordinates in the target postion vector is in the order latitude, longitude and altitude
		self.build1_target = [18.9990965928, 72.0000664814, 10.75]
		self.build2_target = [18.9990965925, 71.9999050292, 22.2]
		self.build3_target = [18.9993675932, 72.0000569892, 10.7]
		self.height_target = 35.00#26.0519618605

		self.Kp = [4000000, 50]
		self.Ki = [0, 0.32]
		self.Kd = [5000000, 80]

		self.build1_error = [0, 0, 0, 0, 0]
		self.build2_error = [0, 0, 0, 0, 0]
		self.build3_error = [0, 0, 0, 0, 0]
		self.prev_error = [0, 0, 0 ,0, 0]
		self.error_sum = [0, 0, 0 ,0, 0]

		self.build1_lat_out = 0
		self.build1_long_out= 0
		self.build1_alt_out = 0

		self.build2_lat_out = 0
		self.build2_long_out = 0
		self.build2_alt_out = 0

		self.build3_lat_out = 0
		self.build3_long_out = 0
		self.build3_alt_out = 0

		self.out_altitude_height = 0

		self.final_latitude = 0
		self.final_longitude = 0
		self.final_altitude = 0

		self.min_value = [1450, 1450, 1000]
		self.max_value = [1550, 1550, 2000]

		self.range1 = 0.0
		self.range2 = 0.0
		self.range3 = 0.0
		self.range4 = 0.0
		self.range5 = 0.0

		self.x = 0.0
		self.y = 0.0
		self.sample_time = 0.060  # in seconds

		self.x_distance = 0.0
		self.y_distance = 0.0

		self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
		self.alt_error = rospy.Publisher('/alt_error',Float32, queue_size=1)
		self.zero_error = rospy.Publisher('/zero_error',Float32, queue_size=1)
		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_finder_callback)
		rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check_callback)
		self.box_attachment_service = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)

	
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		except CvBridgeError as e:
			print(e)
			return
		

	def gps_callback(self, msg):
		self.latitude = msg.latitude
		self.longitude = msg.longitude
		self.altitude = msg.altitude

	def range_finder_callback(self, msg):
		self.range_bottom = msg.ranges[0]
		#rospy.loginfo(self.range_bottom)
	
	def gripper_check_callback(self, msg):
		self.gripper_check = msg.data
		#rospy.loginfo(self.gripper_check)
	
	def marker_detect(self,img_frame):
		img_width = 400
		hfov_rad  = 1.3962634
		focal_length = (img_width/2)/math.tan(hfov_rad/2)
		#print(focal_length)

		#logo_cascade = cv2.CascadeClassifier('data/cascade.xml')

		#img = cv2.imread('data/test_3.png')  # Source image
		#cv2.imshow("image",img)
		gray = cv2.cvtColor(img_frame, cv2.COLOR_BGR2GRAY)

		logo = logo_cascade.detectMultiScale(gray, scaleFactor=1.05)

		for (x, y, w, h) in logo:
			cv2.rectangle(img_frame, (x, y), (x + w, y + h), (255, 255, 0), 2)

		centerx =  x + w/2
		centery =  y + h/2
		#cv2.imshow("window_name", image)	
		#plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
		#plt.show()

		Z_m = self.range_bottom
		self.x_distance = centerx*Z_m/focal_length
		self.y_distance = centery*Z_m/focal_length

	def pid(self):
		self.x = 110692.0702932625 * (self.latitude - 19)
		self.y = -105292.0089353767 * (self.longitude - 72)
		#rospy.loginfo(self.altitude)

		#from initial positon to box
		# Calculating the error
		self.build1_error[0] = self.build1_target[0] - self.latitude
		self.build1_error[1] = self.build1_target[1] - self.longitude
		self.build1_error[2] = self.build1_target[2] - self.altitude
		self.build1_error[3] = self.height_target - self.altitude
		 
		
		self.error_sum[0] = self.error_sum[0] + self.build1_error[0]
		self.error_sum[1] = self.error_sum[1] + self.build1_error[1]
		self.error_sum[2] = self.error_sum[2] + self.build1_error[2]
		self.error_sum[3] = self.error_sum[3] + self.build1_error[3]
		
		self.build1_lat_out = (self.Kp[0] * self.build1_error[0]) + (self.Ki[0] * self.error_sum[0]) + ((self.Kd[0] * (self.build1_error[0] - self.prev_error[0]))/self.sample_time)
		self.build1_long_out = (self.Kp[0] * self.build1_error[1]) + (self.Ki[0] * self.error_sum[1]) + ((self.Kd[0] * (self.build1_error[1] - self.prev_error[1]))/self.sample_time)
		self.build1_alt_out = (self.Kp[1] * self.build1_error[2]) + (self.Ki[1] * self.error_sum[2]) + ((self.Kd[1] * (self.build1_error[2] - self.prev_error[2]))/self.sample_time)

		self.out_altitude_height = (self.Kp[1] * self.build1_error[3]) + (self.Ki[1] * self.error_sum[3]) + ((self.Kd[1] * (self.build1_error[3] - self.prev_error[3]))/self.sample_time)
		# Changing the previous sum value
		self.prev_error[0] = self.build1_error[0]
		self.prev_error[1] = self.build1_error[1]
		self.prev_error[2] = self.build1_error[2]
		self.prev_error[3] = self.build1_error[3]


		self.count += 1
		rospy.loginfo(self.count)

		if self.count < 200:
			rospy.loginfo("path1")
			rospy.loginfo(self.count)
			self.cmd_drone.rcRoll = 1500
			self.cmd_drone.rcPitch = 1500
			self.cmd_drone.rcYaw = 1500
			self.cmd_drone.rcThrottle = 1500 + self.out_altitude_height
		
		if self.count >= 200 and self.count < 500:
			rospy.loginfo("path2")
			self.cmd_drone.rcRoll = 1500 + self.build1_lat_out
			self.cmd_drone.rcPitch = 1500
			self.cmd_drone.rcYaw = 1500
			self.cmd_drone.rcThrottle = 1500 + self.out_altitude_height
			rospy.loginfo(self.count)

		if self.count >= 500 and self.count < 1200:
			rospy.loginfo("path3")
			self.cmd_drone.rcRoll = 1500 #+ self.build1_lat_out
			self.cmd_drone.rcPitch = 1500 + self.build1_long_out
			self.cmd_drone.rcYaw = 1500
			self.cmd_drone.rcThrottle = 1500 + self.out_altitude_height
			rospy.loginfo(self.count)

		if self.count >= 1200 and self.count < 12000:
			rospy.loginfo("path4")
			self.cmd_drone.rcRoll = 1500 + self.build1_lat_out
			self.cmd_drone.rcPitch = 1500 + self.build1_long_out
			self.cmd_drone.rcYaw = 1500
			self.cmd_drone.rcThrottle = 1500 + self.out_altitude_height
			rospy.loginfo(self.count)
			if self.count == 1400:
				marker_detect(self.img)
			rospy.loginfo(self.x_distance)
			rospy.loginfo(self.y_distance)

		if self.cmd_drone.rcRoll > self.max_value[0]:
			self.cmd_drone.rcRoll = self.max_value[0]
		elif self.cmd_drone.rcRoll < self.min_value[0]:
			self.cmd_drone.rcRoll = self.min_value[0]
		else:
			self.cmd_drone.rcRoll = self.cmd_drone.rcRoll

		if self.cmd_drone.rcPitch > self.max_value[1]:
			self.cmd_drone.rcPitch = self.max_value[1]
		elif self.cmd_drone.rcPitch < self.min_value[1]:
			self.cmd_drone.rcPitch = self.min_value[1]
		else:
			self.cmd_drone.rcPitch = self.cmd_drone.rcPitch

		if self.cmd_drone.rcThrottle > self.max_value[2]:
			self.cmd_drone.rcThrottle = self.max_value[2]
		elif self.cmd_drone.rcThrottle < self.min_value[2]:
			self.cmd_drone.rcThrottle = self.min_value[2]
		else:
			self.cmd_drone.rcThrottle = self.cmd_drone.rcThrottle



		self.cmd_pub.publish(self.cmd_drone)
		#publishig errors:
		#self.alt_error.publish(self.error)
		#self.zero_error.publish(0.0)



if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()