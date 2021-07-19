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
import numpy as np
from pyzbar.pyzbar import decode


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        #qr_detect
        rospy.init_node('position_controller') #Initialise rosnode 
        self.img = np.empty([])
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
        # This will contain your image frame from camera
        self.bridge = CvBridge()

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
        self.box_target = [19.0007046575, 71.9998955286, 22.1599967919] #19.0007046575 longitude: 71.9998955286 altitude: 22.1599967919
        self.height_target = 24.0519618605
        self.final_target = [0.0, 0.0, 0.0]   #final target from box qr code in float
        self.targeti = [0.0, 0.0, 0.0]  #final target from box qr code in string

        self.Kp = [4000000, 50]
        self.Ki = [0, 0.32]
        self.Kd = [5000000, 80]

        self.error = [0, 0, 0, 0, 0]
        self.final_error = [0, 0, 0]
        self.prev_error = [0, 0, 0 ,0, 0]
        self.final_prev_error = [0, 0, 0 ,0, 0]
        self.error_sum = [0, 0, 0 ,0, 0]
        self.final_error_sum = [0, 0, 0 ,0]

        self.box_latitude = 0
        self.box_longitude = 0
        self.box_altitude = 0
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

        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.alt_error = rospy.Publisher('/alt_error',Float32, queue_size=1)
        self.zero_error = rospy.Publisher('/zero_error',Float32, queue_size=1)
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.range_finder_callback)
        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check_callback)
        self.box_attachment_service = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
    
    def image_callback(self, data):
        try:
            self.img=self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
        except CvBridgeError as e:
            print(e)
            return
        
        #self.img_decoded = decode(self.img)
        #self.targeti = self.img_decoded[0][0].split(",")
        #rospy.loginfo(type(img_decoded[0][0]))
        #self.final_target[0] = float(self.targeti[0])
        #self.final_target[1] = float(self.targeti[1])
        #self.final_target[2] = float(self.targeti[2])

        #rospy.loginfo(self.final_target)


    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude

    def range_finder_callback(self, msg):
        self.range1 = msg.ranges[0]
        self.range2 = msg.ranges[1]
        self.range3 = msg.ranges[2]
        self.range4 = msg.ranges[3]
        self.range5 = msg.ranges[4]
        #rospy.loginfo(self.range1)
    
    def gripper_check_callback(self, msg):
        self.gripper_check = msg.data
        #rospy.loginfo(self.gripper_check)
    

    def pid(self):
        self.x = 110692.0702932625 * (self.latitude - 19)
        self.y = -105292.0089353767 * (self.longitude - 72)

        #from initial positon to box
        # Calculating the error
        self.error[0]  =self.box_target[0] - self.latitude
        self.error[1] = self.box_target[1] - self.longitude
        self.error[2] = self.box_target[2] - self.altitude
        self.error[3] = self.height_target - self.altitude
         
        
        self.error_sum[0] = self.error_sum[0] + self.error[0]
        self.error_sum[1] = self.error_sum[1] + self.error[1]
        self.error_sum[2] = self.error_sum[2] + self.error[2]
        self.error_sum[3] = self.error_sum[3] + self.error[3]
        
        self.box_latitude = (self.Kp[0] * self.error[0]) + (self.Ki[0] * self.error_sum[0]) + ((self.Kd[0] * (self.error[0] - self.prev_error[0]))/self.sample_time)
        self.box_longitude = (self.Kp[0] * self.error[1]) + (self.Ki[0] * self.error_sum[1]) + ((self.Kd[0] * (self.error[1] - self.prev_error[1]))/self.sample_time)
        self.box_altitude = (self.Kp[1] * self.error[2]) + (self.Ki[1] * self.error_sum[2]) + ((self.Kd[1] * (self.error[2] - self.prev_error[2]))/self.sample_time)

        self.out_altitude_height = (self.Kp[1] * self.error[3]) + (self.Ki[1] * self.error_sum[3]) + ((self.Kd[1] * (self.error[3] - self.prev_error[3]))/self.sample_time)
        # Changing the previous sum value
        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]
        self.prev_error[3] = self.error[3]

        #after picking the box
        self.final_error[0]  =self.final_target[0] - self.latitude
        self.final_error[1] = self.final_target[1] - self.longitude
        self.final_error[2] = self.final_target[2] - self.altitude

        self.final_error_sum[0] += self.final_error[0]
        self.final_error_sum[0] += self.final_error[0]
        self.final_error_sum[0] += self.final_error[0]

        self.final_latitude = (self.Kp[0] * self.final_error[0]) + (self.Ki[0] * self.final_error_sum[0]) + ((self.Kd[0] * (self.final_error[0] - self.final_prev_error[0]))/self.sample_time)
        self.final_longitude = (self.Kp[0] * self.final_error[1]) + (self.Ki[0] * self.final_error_sum[1]) + ((self.Kd[0] * (self.final_error[1] - self.final_prev_error[1]))/self.sample_time)
        self.final_altitude = (self.Kp[1] * self.final_error[2]) + (self.Ki[1] * self.final_error_sum[2]) + ((self.Kd[1] * (self.final_error[2] - self.final_prev_error[2]))/self.sample_time)

        self.final_prev_error[0] = self.final_error[0]
        self.final_prev_error[1] = self.final_error[1]
        self.final_prev_error[2] = self.final_error[2]

        self.count += 1
        #rospy.loginfo(self.count)

        if self.count < 100:
            rospy.loginfo("path1")
            rospy.loginfo(self.count)
            self.cmd_drone.rcRoll = 1500
            self.cmd_drone.rcPitch = 1500
            self.cmd_drone.rcYaw = 1500
            self.cmd_drone.rcThrottle = 1500 + self.out_altitude_height

        if self.count >= 100 and self.count < 500:
            rospy.loginfo("path2")
            rospy.loginfo(self.count)
            self.cmd_drone.rcRoll = 1500 + self.box_latitude
            self.cmd_drone.rcPitch = 1500 + self.box_longitude
            self.cmd_drone.rcYaw = 1500
            self.cmd_drone.rcThrottle = 1500 + self.out_altitude_height


        if self.count >= 500 and self.count < 550:
            rospy.loginfo("going to grip the box")
            self.cmd_drone.rcRoll = 1500
            self.cmd_drone.rcPitch = 1500
            self.cmd_drone.rcYaw = 1500
            self.cmd_drone.rcThrottle = 1500 + self.box_altitude
            if self.count == 400:
                self.img_decoded = decode(self.img)
                self.targeti = str(self.img_decoded[0][0]).split(",")
                self.final_target[0] = float(self.targeti[0])
                self.final_target[1] = float(self.targeti[1])
                self.final_target[2] = float(self.targeti[2])

        
        if self.count >=550 and self.count < 555:
            self.cmd_gripper.activate_gripper = True
            self.box_attachment_service.call(self.cmd_gripper.activate_gripper)
            if self.cmd_gripper.result == True:
                rospy.loginfo("grip successfull")

        if self.count >= 555 and self.count < 650:
            rospy.loginfo("lifting box")
            rospy.loginfo(self.count)
            self.cmd_drone.rcRoll = 1500
            self.cmd_drone.rcPitch = 1500
            self.cmd_drone.rcYaw = 1500
            self.cmd_drone.rcThrottle = 1500 + self.out_altitude_height

        if self.count >= 650 and self.count < 750:
            rospy.loginfo("Moving to final target")
            self.cmd_drone.rcRoll = 1500 #+ self.final_latitude
            self.cmd_drone.rcPitch = 1500 + self.final_longitude
            self.cmd_drone.rcYaw = 1500
            self.cmd_drone.rcThrottle = 1500 + self.out_altitude_height

        if self.count >= 750 and self.count < 1600:
            rospy.loginfo("Moving to final target")
            self.cmd_drone.rcRoll = 1500 + self.final_latitude
            self.cmd_drone.rcPitch = 1500 + self.final_longitude
            self.cmd_drone.rcYaw = 1500
            self.cmd_drone.rcThrottle = 1500 + self.out_altitude_height

        if self.count >= 1600:
            rospy.loginfo("Reached final target")
            self.cmd_drone.rcRoll = 1500 + self.final_latitude
            self.cmd_drone.rcPitch = 1500 + self.final_longitude
            self.cmd_drone.rcYaw = 1500
            self.cmd_drone.rcThrottle = 1500 + self.final_altitude

        if self.count >= 1700:
            self.cmd_gripper.activate_gripper = False
            self.box_attachment_service.call(self.cmd_gripper.activate_gripper)
            if self.cmd_gripper.result == False:
                rospy.loginfo("package was delivered successfully")

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
        self.zero_error.publish(0.0)



if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
