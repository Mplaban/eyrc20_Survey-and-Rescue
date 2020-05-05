#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
from geometry_msgs.msg import PoseArray
import json
import math
import operator
import collections
import time
import csv

class sr_scheduler():

	def __init__(self):

		with open('/home/plaban/catkin_ws/src/survey_and_rescue/scripts/LED_Bonus_Config.tsv') as tsvfile:
			led_config = list(csv.reader(tsvfile, delimiter = "\t"))
		for location in led_config:
			if location[1] == "BASE":
				self.base = location[0]

		rospy.Subscriber('/detection_info',SRInfo,self.detection_callback)	
		rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)
		rospy.Subscriber('/stats_sr', SRDroneStats, self.track_supply)
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		self.decision_pub = rospy.Publisher('/decision_info',SRInfo,queue_size=4)
		
		self.decided_msg = SRInfo()
		self.detected = SRInfo()
		self.serviced = SRInfo()

		self.misc_list = [SRInfo()]
		self.priority_list = [SRInfo()]
		self.garbage = [SRInfo()]
		self.medicine_list = [SRInfo()]
		self.food_list = [SRInfo()]
		self.medicine_list.pop(0)
		self.food_list.pop(0)
		self.garbage.pop(0)
		self.misc_list.pop(0)
		self.priority_list.pop(0)

		self.file = open("/home/plaban/catkin_ws/src/survey_and_rescue/scripts/cell_coords.json","r")
		self.data = json.load(self.file)
		self.baserescue = False
		self.medicine = 0
		self.food = 0
		self.initflag = True
		self.enable = False
		self.baseflag=True

		#timer 
		self.cumulative_duration=0 #variable for storing cumulative duration
		self.duration=0 #variable for storing continuous duration
		self.drone_position = [0.0,0.0,0.0]	
		self.start_time = time.time() 
		self.flag = True

		
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z	

	def track_supply(self,msg):
		self.food = msg.foodOnboard
		self.medicine = msg.medOnboard


	def detection_callback(self,msg):
		self.detected = msg
		if(self.detected.info == "RESCUE"):
			self.priority_list.append(self.detected)
			
		if(self.detected.info == "FOOD" or self.detected.info == "MEDICINE"):
			self.misc_list.append(self.detected)

		if(self.detected.info == "FOOD"):
			self.food_list.append(self.detected)

		if(self.detected.info == "MEDICINE"):
			self.medicine_list.append(self.detected)
			


	def serviced_callback(self,msg):
		# Take appropriate action when either service SUCCESS or FAILIURE is recieved from monitor.pyc
		self.serviced = msg
		if(not self.serviced.location == self.base):
			self.garbage.append(self.serviced)

	def shutdown_hook(self):
		# This function will run when the ros shutdown request is recieved.
		# For instance, when you press Ctrl+C when this is running
		pass

	def publish_decision(self, location, info):
		self.decided_msg.location = location
		self.decided_msg.info = info 
		self.decision_pub.publish(self.decided_msg)

	def calc_dist(self, value, iterable):
		distances = []
		keys = []
		beg = self.data[value.location]
		for k in iterable:
			end = self.data[k.location]
			distances.append(math.sqrt(math.pow(end[0] - beg[0],2) + math.pow(end[1] - beg[1],2)))
			keys.append(k.location)
		ret = dict(zip(keys,distances))
		ret = sorted(ret.items(), key=operator.itemgetter(1))
		dict_ret = collections.OrderedDict(ret)
		return dict_ret.items()[0][0]


	def scheduler(self):
	
		#delete from list whats already served
		for garb in self.garbage:
			for temp in self.priority_list:
				if(garb.location == temp.location):
					self.priority_list.remove(temp)
					self.garbage.remove(garb)
			

			for temp in self.misc_list:
				if(garb.location == temp.location):
					if(temp.info == "FOOD"):
						for status in self.food_list:
							if(status.location == temp.location):
								self.food_list.remove(status)
					elif(temp.info == "MEDICINE"):
						for status in self.medicine_list:
							if(status.location == temp.location):
								self.medicine_list.remove(status)

					self.misc_list.remove(temp)
					self.garbage.remove(garb)

		if(self.baseflag == True):
			if(self.decided_msg.info != "BASE"):
				self.publish_decision(self.base,"BASE")
				self.baseflag=False


		#timer operation#


		if self.drone_position[0]>=(self.data[self.decided_msg.location][0]-0.5) and self.drone_position[0]<=(self.data[self.decided_msg.location][0]+0.5) and self.drone_position[1]>=(self.data[self.decided_msg.location][1]-0.5) and self.drone_position[1]<=(self.data[self.decided_msg.location][1]+0.5) and self.drone_position[2]>=(self.data[self.decided_msg.location][2]-1.0) and self.drone_position[2]<=(self.data[self.decided_msg.location][2]+1.0):
			self.duration = time.time() - self.start_time
			self.flag = True
			if(self.cumulative_duration+self.duration >=3):
				self.flag = False
				self.cumulative_duration=0
		else:
			self.start_time = time.time()
			if(self.flag == True):
				self.cumulative_duration = self.cumulative_duration+self.duration
				self.flag=False



		if(self.baserescue == True):
			if(self.serviced.location == self.decided_msg.location):
				self.baserescue = False
					
		if(self.decided_msg.info == "RESCUE"):
			self.enable = True
			if(self.serviced.location == self.decided_msg.location and self.serviced.info == "FAILURE"):
				self.enable = False
			elif(self.serviced.location == self.decided_msg.location and self.serviced.info == "SUCCESS"):
				self.baserescue = True
				self.enable = False
				self.publish_decision(self.base,"BASE")

				
		if(self.enable == False):
			if(self.food == 0 and self.medicine != 0):
				if(self.medicine_list):
					retval = self.calc_dist(self.decided_msg, self.medicine_list)
					if(self.decided_msg.info != "MEDICINE"):
						self.publish_decision(retval,"MEDICINE")
				else:
					if(self.decided_msg.info != "BASE"):
						self.publish_decision(self.base,"BASE")

			elif(self.medicine == 0 and self.food != 0):
				if(self.food_list):
					retval = self.calc_dist(self.decided_msg, self.food_list)
					if(self.decided_msg.info != "FOOD"):
						self.publish_decision(retval,"FOOD")
				else:
					if(self.decided_msg.info != "BASE"):
						self.publish_decision(self.base,"BASE")

			elif(self.food == 0 and self.medicine == 0):
				if(self.decided_msg.info != "BASE"):
					self.publish_decision(self.base,"BASE")

			if(self.misc_list and not self.priority_list):
				
				retval = self.calc_dist(self.decided_msg, self.misc_list)
				for i in self.misc_list:
					if(i.location == retval):
						if(self.decided_msg not in self.misc_list or self.decided_msg not in self.priority_list):
							if(self.decided_msg.info != "BASE"):
								if(self.decided_msg != i):
									self.publish_decision(retval,i.info)
							elif(self.decided_msg.info == "BASE"):
								if(self.initflag ==True):
									self.publish_decision(retval,i.info)
									self.initflag = False
								else:
									if(self.serviced.location == self.decided_msg.location):
										self.publish_decision(retval,i.info)
					
		

			elif self.priority_list:
				retval = self.calc_dist(self.decided_msg, self.priority_list)
				if(self.decided_msg.info != "RESCUE"):
					if(self.baserescue == False):
						self.publish_decision(retval,"RESCUE")
						self.enable = True


		



def main(args):
	
	sched = sr_scheduler()
	rospy.init_node('sr_scheduler', anonymous=False)
	rospy.on_shutdown(sched.shutdown_hook)
	rate = rospy.Rate(25)
	while not rospy.is_shutdown():
		sched.scheduler()
		rate.sleep()

if __name__ == '__main__':
	main(sys.argv)
