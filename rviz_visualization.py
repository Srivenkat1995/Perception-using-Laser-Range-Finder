#!/usr/bin/env python


import rospy
import math
import sensor_msgs.point_cloud2 as point
import laser_geometry.laser_geometry as lasergeometry
import random
import numpy as np

from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan, PointCloud2
from collections import namedtuple
from numpy import ones,vstack
from numpy.linalg import lstsq
from visualization_msgs.msg import Marker



#GLobal INtializations

sine = np.array((361,1))
cosine = np.array((361,1))
ranges = np.zeros((361,2))

xpoints = []
ypoints = [] 

point_list = []
 
def scandata(msg):
		global point_list
		pointcloud_msg = lasergeometry.LaserProjection().projectLaser(msg)
		point_list = point.read_points_list(pointcloud_msg)

		

def laserscanandransaclinesvisualization():
		rate = rospy.Rate(10)
		markerLine_publisher = rospy.Publisher("visualization_marker", Marker, queue_size=10)
	
		pointcloud_publisher = rospy.Publisher("converted_pc", PointCloud2, queue_size = 10)	
		rospy.Subscriber("/base_scan", LaserScan, scandata)
			
		print ("Laser DATA subscribed")			
		
		while not rospy.is_shutdown():
			
			implementingransac()
			#visualizingransaclines(markerLine_publisher)
			markerLine = Marker()
	
			markerLine.header.frame_id = "base_laser_link"
    			markerLine.type = Marker.LINE_LIST
    			markerLine.action = Marker.ADD
			markerLine.header.stamp = rospy.Time.now()
    			
			
	# marker scale
    	
			markerLine.scale.x = 0.1
   			markerLine.scale.y = 0.0
    			markerLine.scale.z = 0.0

    	# marker color
   			markerLine.color.a = 1.0
    			markerLine.color.r = 1.0
    			markerLine.color.g = 1.0
    			markerLine.color.b = 0.0

   	# marker orientaiton
    			markerLine.pose.orientation.x = 0.0
    			markerLine.pose.orientation.y = 0.0
    			markerLine.pose.orientation.z = 0.0
    			markerLine.pose.orientation.w = 1.0
			

			
			   	# Appending Points obtained from Inliers.

			for i in range(1,len(xpoints)):
		
				point_line = Point()
		
				point_line.x = xpoints[i]
				point_line.y = ypoints[i]
				print(point_line)
				markerLine.points.append(point_line)
			
			markerLine.lifetime = rospy.Duration()
			#print (markerLine)
			markerLine_publisher.publish(markerLine)
			rate.sleep()

		
def implementingransac():
	
		
		global xpoints 
		global ypoints
		
		new_x = []
		new_y = []
			
		
		for i in range(len(point_list)):
			new_x.append(point_list[i].x)
			new_y.append(point_list[i].y)
		no_of_iterations = len(new_x) - 10
		#print(new_x)	 	
		for i in range(5):
			
			inliers = []
			outliers = []
		
			max_inlier_x = 0 
			max_inlier_y = 0 
			min_inlier_x = 0 
			min_inlier_y = 0
			for j in range(no_of_iterations):
				
				inlier_max = []
				outlier_max = []
				slope = 0.0
				constant = 0.0
				# CHOOSING TWO RANDOM NOS 

				no1 = random.randint(0 , len(new_x) - 1)
				no2 = random.randint(0 , len(new_x) - 1)
				
				# Assinging the no with corresponding points

				if no1 == no2:
					continue
				
				
				x1 = new_x[no1]
				y1 = new_y[no1]
				x2 = new_x[no2]
				y2 = new_y[no2]
		

				if x1 == 0 and x2 == 0:
					continue

				# Finding the distances from remaining points C(x3,y3) to the A(x1,y1) and B(x2 , y2)
		
				for k in range(len(new_x) -1 ):
					
					x3 = new_x[k]
					y3 = new_y[k]
					if x3 == 0 and y3 == 0:
						continue
					if (x2-x1 == 0):
						continue
					slope = (y2-y1) / (x2-x1)
					constant = y1 - slope * (x1)

					distance = abs( (slope * x3) + ((-1) * y3) + constant ) / math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
					
					print (distance)
					if distance < 0.1:
						inlier_max.append(k)
					else:
						outlier_max.append(k)
	
				
				if len(inliers) < len(inlier_max):
		
					inliers = inlier_max
					outliers = outlier_max
				
					
					
		
 			if len(inliers) > 3:
				
				for i in range(len(inliers) - 1):				
					xpoints.append(new_x[i])
					ypoints.append(new_y[i])
				print("Points appended")
				
				
			if len(point_list) < 2:
				break



if __name__ == "__main__":
	try:
		rospy.init_node("Ransac_implementaion")
		print("Node Intiated")
		
		laserscanandransaclinesvisualization()
		print("Laser Scan and RANSAC Lines Visualization Function is called")
		
	
	except  rospy.ROSInterruptException:
		pass



