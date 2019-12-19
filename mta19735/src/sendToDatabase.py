#!/usr/bin/env python

import rospy
import requests
import json 
import yaml

from std_msgs.msg import Float32, String,UInt32
from geometry_msgs.msg import Pose2D, Twist, Point
from sensor_msgs.msg import Imu
from gps_common.msg import GPSFix
from numpy import pi, sign

from rospy_message_converter import json_message_converter
from std_msgs.msg import String

import time
import os

inputArrays = [None,None,None,None,None,None]

def gpscallback(_data):
	callback(_data,0)

def imucallback(_data):
	callback(_data,1)

def timecallback(_data):
	callback(_data,2)

def windcallback(_data):
	callback(_data,3)

def sailcallback(_data):
	callback(_data,4)

def ruddercallback(_data):
	callback(_data,5)

def callback(_data,sensorType):
	global inputArrays

	# Using a library to convert ROS message into readable JSON format
	inputArrays[sensorType]=json_message_converter.convert_ros_message_to_json(_data)

	complete = 1
	# If any of the sensors has yet been tracked
	for sensor in inputArrays:
		if sensor == None:
			complete = 0
	# Should only send to server once all sensors has been tracked
	if complete == 1:
		try:
			# Converting JSON content to string and sending it in its respective field
			r = requests.post('http://192.168.1.100:3001/boat', data={"gps":str(inputArrays[0]),"imu":str(inputArrays[1]),"time":str(inputArrays[2]),"wind":str(inputArrays[3]),"sail":str(inputArrays[4]),"rudder":str(inputArrays[5])}, timeout=3.9)
			print("SUCCESS: sensor input sent to database")
			inputArrays = [None,None,None,None,None,None]
		except requests.exceptions.RequestException as e:
			print("FAILURE: sensor input failed to send to database")
			inputArrays = [None,None,None,None,None,None]

def sendToDatabase():
	print("starting script")
	rospy.init_node('testSubscriber', anonymous=True)
	rospy.Subscriber("sailboat/GPS/fix", GPSFix, gpscallback)
	rospy.Subscriber("sailboat/IMU", Imu, imucallback)
	rospy.Subscriber("sailboat/Time", UInt32, timecallback)
	rospy.Subscriber("sailboat/wind", Pose2D, windcallback)
	rospy.Subscriber("sailboat/sail", Float32, sailcallback)
	rospy.Subscriber("sailboat/rudder", Float32, ruddercallback)
	print("all topics subscribed to")

  # spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
	print('stopped sending data to database')


if __name__ == '__main__':
	sendToDatabase()

