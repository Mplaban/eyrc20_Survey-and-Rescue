#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pickle
import imutils
import copy
import numpy as np
import itertools
import json


class sr_determine_rois():

	def __init__(self):

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/usb_cam/image_rect",Image,self.image_callback)
		self.img = None
		self.img_copy = None
		self.roi_list = []
		self.block_name_list = ["A1", "A2", "A3", "A4", "A5", "A6", "B1", "B2", "B3", "B4", "B5", "B6", "C1", "C2", "C3", "C4", "C5", "C6","D1", "D2", "D3", "D4", "D5", "D6", "E1", "E2", "E3", "E4", "E5", "E6", "F1", "F2", "F3", "F4", "F5", "F6"]
		self.ROI = []
		self.kk = None
	
	# CV_Bridge acts as the middle layer to convert images streamed on rostopics to a format that is compatible with OpenCV
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "mono8")
		except CvBridgeError as e:
			print(e)


	'''You will need to implement an image processing algorithm to detect the Regions of Interest (RoIs)
	The standard process flow is:
	i)		Standard pre-processing on the input image (de-noising, smoothing etc.)
	ii)		Contour Detection
	iii)	Finding contours that are square, polygons with 4 vertices
	iv)		Implementing a threshold on their area so that only contours that are the size of the cells remain'''
	 
	global get_contour_precedence
	def get_contour_precedence(contour, cols):
		tolerance_factor = 10
		origin = cv2.boundingRect(contour)
		return ((origin[0] // tolerance_factor) * tolerance_factor) * cols + origin[1]

	def detect_rois(self):
		self.img_copy = self.img.copy()
		ratio = self.img_copy.shape[0] / 300.0
		#self.img_copy = imutils.resize(self.img_copy, height = 300)

		blur = cv2.bilateralFilter(self.img_copy, 11, 17, 17)
		ret,threshold = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
		threshold = cv2.GaussianBlur(threshold,(5,5),8)

		_,self.cnts, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		

		

	def sort_rois(self):
		# Add your Code here
		self.cnts = sorted(self.cnts, key = cv2.contourArea, reverse = True)[:37]
		self.cnts.pop(0)
		self.cnts.sort(key=lambda x:get_contour_precedence(x, self.img.shape[0]))

	def query_yes_no(self, question, default=None):
		"""Ask a yes/no question via raw_input() and return their answer.

		"question" is a string that is presented to the user.
		"default" is the presumed answer if the user just hits <Enter>.
		It must be "yes" (the default), "no" or None (meaning
		an answer is required of the user).

		The "answer" return value is True for "yes" or False for "no".
		"""
		valid = {"yes": True, "y": True, "ye": True,"no": False, "n": False}
		if default is None:
			prompt = " [Y/N]:\t"
		elif default == "yes":
			prompt = " [Y/N]:\t"
		elif default == "no":
			prompt = " [Y/N]:\t"
		else:
			raise ValueError("Invalid default answer: '%s'" % default)

		while True:
			sys.stdout.write(question + prompt)
			choice = raw_input().lower()
			if default is not None and choice == '':
				return valid[default]
			elif choice in valid:
				return valid[choice]
			else:
				sys.stdout.write("\nPlease respond with 'yes' or 'no' ""(or 'y' or 'n').\n")

	def save_rois(self):
		#Add your code here
		for j,c in enumerate(self.cnts):
			x,y,w,h = cv2.boundingRect(c)
			start = (x,y)
			end = (x+w, y+h)
			self.kk = cv2.rectangle(self.img,start,end,(0,0,255),2)
			self.roi_list.append(list([x,y,x+w,y+h]))

		ROI = dict(zip(self.block_name_list,self.roi_list))
		with open('SavedROI.json', mode='w') as outfile:
			json.dump(ROI,outfile)
		print("all done")

					   

	#You may optionally implement this to display the image as it is displayed in the Figure given in the Problem Statement
	def draw_cell_names(self, img):
		#Add your code here
		cv2.drawContours(self.img_copy,self.cnts,-1,(255, 255, 255),3)

		for i in range(len(self.cnts)):
			self.img_copy = cv2.putText(self.img_copy, str(i+1), cv2.boundingRect(self.cnts[i])[:2], cv2.FONT_HERSHEY_COMPLEX, 1, [50])

		cv2.imshow("kk",self.kk)
		cv2.waitKey(0)
		cv2.destroyAllWindows()
		
def main(args):
	#Sample process flow
	try:
		rospy.init_node('sr_roi_detector', anonymous=False)
		r =	sr_determine_rois()
		while True:
			if r.img is not None:
				r.detect_rois()
				if(len(r.cnts)<36):
					new_thresh_flag = r.query_yes_no("36 cells were not detected, do you want to change ##Enter tweaks, this is not necessary##?")
					if(new_thresh_flag):
						#Change settings as per your desire

						k=1
					else:
						continue
				else:
					satis_flag = r.query_yes_no("Are you satisfied with the currently detected ROIs?")
					if(satis_flag):
						r.sort_rois()
						r.save_rois()
						cv2.destroyAllWindows()
						break
					else:
						k=2
						#Change more settings
		# r.draw_cell_names(r.img) # Again, this is optional
		r.draw_cell_names(r.img_copy) # Again, this is optional
	except KeyboardInterrupt:
		cv2.destroyAllWindows() 

if __name__ == '__main__':
	main(sys.argv)
	
