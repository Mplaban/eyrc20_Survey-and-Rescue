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
from cv_bridge import CvBridge, CvBridgeError
import random
import json
import imutils
import copy
import time


class sr_determine_colors():

	def __init__(self):
		self.img = None
		self.img_copy = None
		self.detect_info_msg = SRInfo()
		self.message = SRInfo()
		self.bridge = CvBridge()
		self.detect_pub = rospy.Publisher("/detection_info",SRInfo,queue_size=10) 
		self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color",Image,self.image_callback)
		self.serviced_sub = rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)

		self.hsv = None
		self.blue_mask = None
		self.red_mask = None
		self.green_mask = None

		self.isBlue = False
		self.isRed = False
		self.isGreen = False

		self.roi_list = ["A1", "A2", "A3", "A4", "A5", "A6", "B1", "B2", "B3", "B4", "B5", "B6", "C1", "C2", "C3", "C4", "C5", "C6","D1", "D2", "D3", "D4", "D5", "D6", "E1", "E2", "E3", "E4", "E5", "E6", "F1", "F2", "F3", "F4", "F5", "F6"]

	def load_rois(self, file_path = '/home/plaban/catkin_ws/src/survey_and_rescue/scripts/SavedROI.json'):
		try:
			with open(file_path, 'rb') as input:
				self.rect_list = json.load(input)
		except IOError, ValueError:
			print("File doesn't exist or is corrupted")


	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)


	def serviced_callback(self, msg):
		self.message = msg
		time.sleep(0.1)
		if(len(self.message.location)):
			self.check_whether_lit()
			time.sleep(0.1)
			self.find_color_contour_centers()
			self.roi_list.append(self.message.location)
	

	def check_contours(self,x,y,s,info,cchh):
		val = self.rect_list[s]
		if(val[0]<x<val[2] and val[1]<y<val[3]):
			self.detect_info_msg.location = s
			self.detect_info_msg.info = info
			self.detect_pub.publish(self.detect_info_msg)
			try:
				self.roi_list.remove(s)
			except ValueError:
				pass


	def find_color_contour_centers(self):
		for roi in self.roi_list:
		

			if(self.isBlue == True):
				t1,thresh_blue = cv2.threshold(self.blue_mask, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
				_,cnts_blue,_ = cv2.findContours(thresh_blue,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
					#cv2.drawContours(self.img_copy,cnts_blue,-1,(255, 0, 0),2)
				for i in cnts_blue:
					M_B = cv2.moments(i)
					try:
						x = int(M_B["m10"]/M_B["m00"])
						y = int(M_B["m01"]/M_B["m00"])
					except ZeroDivisionError:
						pass
					self.check_contours(x,y,roi,"MEDICINE","B")


			if(self.isRed == True):
				t2,thresh_red = cv2.threshold(self.red_mask, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
				_,cnts_red,_ = cv2.findContours(thresh_red,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
				#cv2.drawContours(self.img_copy,cnts_red,-1,(0, 0, 255),2)
				for i in cnts_red:
					M_R = cv2.moments(i)
					try:
						x = int(M_R["m10"]/M_R["m00"])
						y = int(M_R["m01"]/M_R["m00"])
					except ZeroDivisionError:
						pass
					self.check_contours(x,y,roi,"RESCUE","R")

							

			if(self.isGreen == True):
				t3,thresh_green = cv2.threshold(self.green_mask, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
				_,cnts_green,_ = cv2.findContours(thresh_green,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
				#cv2.drawContours(self.img_copy,cnts_green,-1,(0, 255, 0),2)
				for i in cnts_green:
					M_G = cv2.moments(i)
					try:
						x = int(M_G["m10"]/M_G["m00"])
						y = int(M_G["m01"]/M_G["m00"])
					except ZeroDivisionError:
						pass
					self.check_contours(x,y,roi,"FOOD","G")


	
	def check_whether_lit(self):

		self.img_copy = self.img.copy()

		kernel = np.ones((5,5), np.uint8)

		self.hsv = cv2.cvtColor(self.img_copy,cv2.COLOR_BGR2HSV)

		lower_blue = np.array([110,125,150],dtype = np.uint8)
		upper_blue = np.array([130,255,255],dtype = np.uint8)

		lower_red = np.array([160,172,235],dtype = np.uint8)
		upper_red = np.array([180,255,255],dtype = np.uint8)

		lower_green = np.array([45,110,150],dtype = np.uint8)
		upper_green = np.array([70,255,255],dtype = np.uint8)

		blue = cv2.inRange(self.hsv,lower_blue ,upper_blue)
		red = cv2.inRange(self.hsv, lower_red, upper_red)
		green = cv2.inRange(self.hsv, lower_green, upper_green)

		#time.sleep(2)

		self.blue_mask = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, kernel)
		self.red_mask = cv2.morphologyEx(red, cv2.MORPH_CLOSE, kernel)		
		self.green_mask = cv2.morphologyEx(green, cv2.MORPH_CLOSE, kernel)

		self.blue_mask = cv2.dilate(self.blue_mask,kernel,iterations = 4)
		self.green_mask = cv2.dilate(self.green_mask,kernel,iterations = 5)
		self.red_mask = cv2.dilate(self.red_mask,kernel,iterations = 4)



		if(cv2.countNonZero(self.blue_mask) != 0):
			self.isBlue = True
		else:
			self.isBlue = False

		if(cv2.countNonZero(self.green_mask) != 0):
			self.isGreen = True
		else:
			self.isGreen = False

		if(cv2.countNonZero(self.red_mask) != 0):
			self.isRed = True
		else:
			self.isRed = False
		
		#print(self.isRed)
		#print(self.isGreen)
		#print(self.isBlue)

def main(args):
	
	try:
		rospy.init_node('sr_beacon_detector', anonymous=False)
		s = sr_determine_colors()
		'''You may choose a suitable rate to run the node at.
		Essentially, you will be proceesing that many number of frames per second.
		Since in our case, the max fps is 30, increasing the Rate beyond that
		will just lead to instances where the same frame is processed multiple times.'''
		rate = rospy.Rate(25)
		s.load_rois()
		while s.img is None:
			pass
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
	while not rospy.is_shutdown():
		try:
			s.check_whether_lit()
			s.find_color_contour_centers()
			#rate.sleep()
		except KeyboardInterrupt:
			cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
