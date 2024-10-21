#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2 as cv
from aruco_detection import Detected_ArUco_markers
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):
	Detected_ArUco_markers = {}	

	gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
	parameters = aruco.DetectorParameters_create()
	corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
	list1 = []
	list2 = []
	for i in ids:
		i = int(i)
		list1.append(i)
        
	for j in corners:
		list2.append(j)

	for i in list1:
		for j in list2:
			Detected_ArUco_markers[i] = [j]
			list2.remove(j)
			break
	print(Detected_ArUco_markers)


def Calculate_orientation_in_degree(Detected_ArUco_markers,img):

	ArUco_marker_angles = {}
	## enter your code here ##
	key = list(Detected_ArUco_markers.keys())
	gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
	parameters = aruco.DetectorParameters_create()
	corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
	_, bw = cv.threshold(gray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
	contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
	value = list(Detected_ArUco_markers.values())
	list3 = []
	midpoint = []
	contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
 
	for i,c in enumerate(contours):

		area = cv.contourArea(c)

		if area < 3700 or 100000 < area:
			continue

		rect = cv.minAreaRect(c)
		center = (int(rect[0][0]),int(rect[0][1])) 
		list3.append(center)
		

		for i in value:
			for j in i:
				mp = (j[0]+j[1])/2
				midpoint.append(mp)
	
	res = []
	for i in midpoint:
		for j in i:
			j = int(j)
			res.append(j)
	list4 = []
	res = [(res[i],res[i+1]) for i in range(0,len(res)//2,2)]
	for i in list3:
		for j in res:
			iradian = math.atan2(i[1]-j[1],i[0]-j[0])
			mydegrees = int(math.degrees(iradian))
		list4.append(mydegrees)
    
	list4 = [abs(ele) for ele in list4]
	for i in key:
		for j in list4:
			ArUco_marker_angles[i] = j
			list4.remove(j)
			break

	return ArUco_marker_angles	## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):

	list1 = []
	list2 = []
	list3 = []
	list5 = []
	gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
	parameters = aruco.DetectorParameters_create()
	corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
	for j in corners:
		list2.append(j)
	for x in ids:
		x = int(x)
		list5.append(x)
	_, bw = cv.threshold(gray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
	contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
	
	for i,c in enumerate(contours):

		area = cv.contourArea(c)

		if area < 3700 or 100000 < area:
			continue

		rect = cv.minAreaRect(c)
		center = (int(rect[0][0]),int(rect[0][1]))
		list3.append(center)

		#red dot in the centre
		cv.circle(img, (center),5,(0,0,255),-1)

		for i in list2:
			for j in i:
				mp = (j[0]+j[1])/2
				list1.append(mp)

	res = []
	for i in list1:
		for j in i:
			j = int(j)
			res.append(j)

	res = [(res[i],res[i+1]) for i in range(0,len(res)//2,2)]

	# blue line from center to midpoint
	for i in list3:
		for j in res:
			cv.line(img, (j[0],j[1]),(i[0],i[1]),(255,0,0),4)
			res.remove(j)
			break

	#Aruco id code
	for i in list5:
		font = cv.FONT_HERSHEY_COMPLEX
		for j in list3:
			cv.putText(img, str(i),(j[0],j[1]), font, 1, (0,0,255),5,cv.LINE_AA)
			list3.remove(j)
			break

	#colorfull dots on everycorner
	edu = [] #corner integers
	for i in list2:
		for j in i:
			for k in j:
				for l in k:
					l = int(l)
					edu.append(l)
	
	list4 = []
	res = [(res[i],res[i+1]) for i in range(0,len(res)//2,2)]
	for i in list3:
		for j in res:
			iradian = math.atan2(i[1]-j[1],i[0]-j[0])
			mydegrees = int(math.degrees(iradian))
		list4.append(mydegrees)
	list4 = [abs(ele) for ele in list4]

	edu = [(edu[i],edu[i+1]) for i in range(0,len(edu),2)]

	alen = [edu[i:i+4] for i in range(0, len(edu), 4)]

	for i in alen:
		cv.circle(img, i[0],5,(124,125,125),-1)
		cv.circle(img, i[1],5,(0,255,0),-1)
		cv.circle(img, i[2],5,(180,105,255),-1)
		cv.circle(img, i[3],5,(255,255,255),-1)
	
	

	for i in list4:
		font = cv.FONT_HERSHEY_COMPLEX
		for j in alen:
			cv.putText(img, str(i),(j[3]), font, 1, (0,255,0),2,cv.LINE_AA)
			alen.remove(j)
			break
	
	#displaying the image
	return img