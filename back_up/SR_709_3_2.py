#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
									
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import json




class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		self.file = open("/home/plaban/catkin_ws/src/survey_and_rescue/scripts/cell_coords.json","r")
		self.data = json.load(self.file)
		""" variables for cell coordinate """
		self.j=1 
		self.flag=True
		self.char = "A"
		""" setpoint """
		self.setpoint=self.data["A1"]
		self.start_time = time.time() 
		self.s = "A1" #this string stores the current cell coordinate
		self.cumulative_duration=0 #variable for storing cumulative duration
		self.duration=0 #variable for storing continuous duration

		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. 
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	


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


		#initial setting of Kp, Kd and ki for [pitch, roll, throttle]


		self.Kp = [45.88,29.62,55.88]
		self.Ki = [0.00075,0.00013,0.0395]    
		self.Kd = [1284.2,842.8,995.2]




		#pid variables

		self.error = [0,0,0]
		self.prev_values = [0,0,0]
		self.min_values = [1400,1400,1000]
		self.max_values = [1700,1700,2000]


		self.Iterm = [0,0,0]
		self.out =[0,0,0] 
		self.lasttime=0

		self.sample_time = 0.033 # in seconds


		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		self.alt_error = rospy.Publisher('/alt_error',Float64,queue_size=1)
		self.pit_error = rospy.Publisher('/pitch_error',Float64,queue_size=1)
		self.roll_error = rospy.Publisher('/roll_error',Float64,queue_size=1)



		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone 
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)


	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z


	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	def pitch_set_pid(self,pit):
		self.Kp[1] = pit.Kp * 0.06
		self.Ki[1] = pit.Ki * 0.008
		self.Kd[1] = pit.Kd * 0.3

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp * 0.06
		self.Ki[0] = roll.Ki * 0.008
		self.Kd[0] = roll.Kd * 0.3


	def pid(self):
	
		now = time.time()

		timechange = now - self.lasttime

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

			if self.cmd.rcRoll > self.max_values[0]:
				self.cmd.rcRoll = self.max_values[0]

			if self.cmd.rcRoll < self.min_values[0]:
				self.cmd.rcRoll = self.min_values[0]

			if self.cmd.rcPitch > self.max_values[1]:
				self.cmd.rcPitch = self.max_values[1]

			if self.cmd.rcPitch < self.min_values[1]:
				self.cmd.rcPitch = self.min_values[1]

			if self.cmd.rcThrottle > self.max_values[2]:
				self.cmd.rcThrottle = self.max_values[2]

			if self.cmd.rcThrottle < self.min_values[2]:
				self.cmd.rcThrottle = self.min_values[2]

			#storing previous error values
			self.prev_values[0] = self.error[0]
			self.prev_values[1] = self.error[1]
			self.prev_values[2] = self.error[2]

			self.lasttime=now

			
		#print("Drone at  "+self.s+"\r"), #printing current drone position
		

		if self.drone_position[0]>=(self.setpoint[0]-0.5) and self.drone_position[0]<=(self.setpoint[0]+0.5) and self.drone_position[1]>=(self.setpoint[1]-0.5) and self.drone_position[1]<=(self.setpoint[1]+0.5) and self.drone_position[2]>=(self.setpoint[2]-1.0) and self.drone_position[2]<=(self.setpoint[2]+1.0):
			self.duration = time.time() - self.start_time
			print("Drone at position  " + self.s + "   "+str(round(self.cumulative_duration+self.duration,2))+"  seconds...")
			self.flag = True
			if(self.cumulative_duration+self.duration >=3):
				self.flag = False
				if(self.j<6):
					self.j= self.j+1 #increment cell number
					self.s = str(self.char)+str(self.j)
					self.cumulative_duration=0
					self.setpoint = self.data[self.s]
					self.start_time = time.time() #refresh the starting time of timer
				elif self.j>=6 : 
					if(self.s == "F6"):
						print("DISARMED")
						self.disarm()
						self.cumulative_duration =0
					else:
						self.char = chr(ord(self.char[0])+1)
						self.j=1 #reset cell number 
						self.s = str(self.char)+str(self.j) #increment cell letter notation
						self.cumulative_duration=0
						self.setpoint = self.data[self.s] 
						self.start_time = time.time() #refresh the starting time of timer

			
		else:
			#print(str(round(self.cumulative_duration,2)))
			self.start_time = time.time()
			if(self.flag == True):
				self.cumulative_duration = self.cumulative_duration+self.duration
				self.flag=False
					
		"""publish the errors and drone commands"""
		self.command_pub.publish(self.cmd)
		self.pit_error.publish(self.error[0])
		self.roll_error.publish(self.error[1])
		self.alt_error.publish(self.error[2])



if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(30) #rospy refresh rate 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()
