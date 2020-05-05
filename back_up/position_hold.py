#!/usr/bin/env python

'''
*Team Id : #SR_709
*Author List : Pratyush Kumar Parida, Plaban Mohapatra ,Ritik Sen
*Filename:Position_hold.py
*Theme:Survey and Rescue-eYRC
*Functions: _init_(), disarm(),arm(),whycon_callback(),altitude_set_pid(),pitch_set_pid(),roll_set_pid(),setpoint_set(),pid()
*Global Variables: NONE

'''
from __future__ import print_function
from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
import json
import math
import operator
import collections
import rospy
import time
import csv



class Edrone():
	
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		with open('/home/plaban/catkin_ws/src/survey_and_rescue/scripts/LED_Bonus_Config.tsv') as tsvfile:
			led_config = list(csv.reader(tsvfile, delimiter = "\t"))
		for location in led_config:
			if location[1] == "BASE":
				self.base = location[0]

		self.drone_position = [0.0,0.0,0.0] #drone_position: This corresponds to current position of drone [x,y,z]. 
		self.decided_msg = SRInfo() #decided_msg:
		 
		self.file = open("/home/plaban/catkin_ws/src/survey_and_rescue/scripts/cell_coords.json","r")
		self.data = json.load(self.file)

		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		self.setpoint = self.data[self.base] #setpoint: Initial setpoint for the to take off and attain the required height i.e 1.5 feet above ground as mentioned in the rulebook

													


		#values of Kp, Kd and ki for [pitch,roll,throttle]
		
		self.Kp = [27.32,31.72,55.88]
		self.Ki = [0.0075,0.055,0.0395]    
		self.Kd = [1184.2,990.8,765.2] #purana

		#self.Kp = [45.88,29.62,55.88]
		#self.Ki = [0.00075,0.0013,0.0395]    
		#self.Kd = [1284.2,842.8,995.2]

		#self.Kp = [36.8,22.72,54.16]  
		#self.Ki = [0.004,-0.008,0.008]    
		#self.Kd = [1275.2,1376.4,760]#submitable

		#self.Kp = [27.32,22.72,54.16]  
		#self.Ki = [0.075,-0.005,0.008]    
		#self.Kd = [1325.2,1350.4,760]#submittable 1


		self.error = [0,0,0]#error: Used for storing error values in x,y and z axis.[x,y,z]
		self.prev_values = [0,0,0]#prev_values: storing previous error after the last pid calculation in [x,y,z] axis.
		self.min_values = [1400,1400,1300]#min_values: minimum threshold values of output[pitch,roll,throttle] 
		self.max_values = [1700,1700,1800]#max_values: maximum threshold values of output[pitch,roll,throttle] 

		self.Iterm = [0,0,0]#Iterm: This store the intergral term calculation of the pid algorithm which is then added to get the correction pid value.[pitch,roll,throttle]
		self.out =[0,0,0] #out:Correction values after pid algorithm is performed [pitch,roll,throttle].
		self.lasttime=0

 
		self.sample_time = 0.033 # sample_time:This varible stores the time interval of pid controller operation in seconds.


		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		self.alt_error = rospy.Publisher('/alt_error',Float64,queue_size=1)
		self.pit_error = rospy.Publisher('/pitch_error',Float64,queue_size=1)
		self.roll_error = rospy.Publisher('/roll_error',Float64,queue_size=1)
		self.zero=rospy.Publisher('/zero',Float64,queue_size=1)



		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/decision_info',SRInfo,self.setpoint_set)


		self.arm() # ARMING THE DRONE


	def disarm(self):
	# Disarming condition of the drone
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.3)


	def arm(self):
	# Arming condition of the drone
		self.disarm()
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(0.3)


	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y 
		self.drone_position[2] = msg.poses[0].position.z


	# Callback function for /pid_tuning_altitude
	'''
	* Function Name: altitude_set_pid()
	* Input:None
	* Output:None
	* Logic:This function gets executed each time when /tune_pid publishes /pid_tuning_altitude and set pid values of throttle. 
	* Example Call:It is a callback function called automatically when /tune_pid publishes /pid_tuning_altitude topic
	'''
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.08 
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.4


	def pitch_set_pid(self,pit):
		self.Kp[0] = pit.Kp * 0.08
		self.Ki[0] = pit.Ki * 0.0002
		self.Kd[0] = pit.Kd * 0.4

	def roll_set_pid(self,roll):
		self.Kp[1] = roll.Kp * 0.08
		self.Ki[1] = roll.Ki * -0.0008
		self.Kd[1] = roll.Kd * 0.4


	def setpoint_set(self,msg):
		self.decided_msg = msg #decided_msg: This variable stores the msg published on the /decision_info topic
		self.setpoint = self.data[self.decided_msg.location] #setpoint: This variable stores the setpoint(in whycon cordintes) of the next decided cell location published, by acessing the json data list of stored whycon cordinates of the cells.

	

	def pid(self):
	

		now = time.time()#now: Store the current time.

		timechange = now - self.lasttime#timechange: Calculates the time change between current time and since last pid values were calculated.

		if timechange > self.sample_time:
			#roll_pid ie y-axis
			self.error[1] =  self.setpoint[1] - self.drone_position[1]  
			chg_errorp = self.error[1] - self.prev_values[1]
			self.Iterm[1] += self.Ki[1] * self.error[1]

			self.out[1]=self.Kp[1] * self.error[1] +self.Iterm[1] + self.Kd[1] * chg_errorp


			#pitch_pid ie x-axis 
			self.error[0] =  self.setpoint[0] - self.drone_position[0] 
			chg_errorR = self.error[0] - self.prev_values[0]
			self.Iterm[0] += self.Ki[0] * self.error[0]

			self.out[0]=self.Kp[0] * self.error[0] +self.Iterm[0] + self.Kd[0] * chg_errorR


			#throttle_pid ie z-axis
			self.error[2] = self.drone_position[2] - self.setpoint[2]
			chg_errorT = self.error[2] - self.prev_values[2]
			self.Iterm[2] += self.Ki[2] * self.error[2]

			self.out[2]=self.Kp[2] * self.error[2] +self.Iterm[2] + self.Kd[2] * chg_errorT

			#setting pid output on respective values
			self.cmd.rcPitch = 1500 + self.out[0]
			self.cmd.rcRoll = 1500 + self.out[1]
			self.cmd.rcThrottle = 1500 + self.out[2]

			#print(self.cmd.rcRoll)

			if self.cmd.rcRoll > self.max_values[1]:
				self.cmd.rcRoll = self.max_values[1]

			if self.cmd.rcRoll < self.min_values[1]:
				self.cmd.rcRoll = self.min_values[1]

			if self.cmd.rcPitch > self.max_values[0]:
				self.cmd.rcPitch = self.max_values[0]

			if self.cmd.rcPitch < self.min_values[0]:
				self.cmd.rcPitch = self.min_values[0]

			if self.cmd.rcThrottle > self.max_values[2]:
				self.cmd.rcThrottle = self.max_values[2]

			if self.cmd.rcThrottle < self.min_values[2]:
				self.cmd.rcThrottle = self.min_values[2]

			#storing previous error values
			self.prev_values[0] = self.error[0]
			self.prev_values[1] = self.error[1]
			self.prev_values[2] = self.error[2]

			self.lasttime=now #lasttime: This stores the time in seconds when last pid output values were calculated. 

	

	#publishing data
		self.command_pub.publish(self.cmd)#publishes the drone command on /drone_command
		self.pit_error.publish(self.error[0])#publishes the error in pitch on /pitch_error
		self.roll_error.publish(self.error[1])#publishes the error in pitch on /roll_error
		self.alt_error.publish(self.error[2])#publishes the error in pitch on /alt_error
		self.zero.publish(0)



if __name__ == '__main__':

	e_drone = Edrone() #edrone: Declares object of Edrone class.
	rospy_rate = rospy.Rate(30) #rospy_rate: Specifies the rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid() #pid() function is called in a loop till rospy.is_shutdown() is False.
		rospy_rate.sleep()
