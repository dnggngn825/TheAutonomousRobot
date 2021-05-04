#!/usr/bin/env python
# -*- coding: utf-8 -*-

# /* ==================================================
# AUTHORSHIP STATEMENT
# The University of Melbourne
# School of Engineering
# ELEN90090: Autonomous Clinic Systems
# Author: Hai Dang Nguyen (860308)
# ================================================== */

# /**
#  * @brief The node read the data from the csv file of the path that we generate
#  * then pushed to gitlab
#  * 
#  */

# Import the ROS-Python package
import serial
import rospy
import csv
import os
import rospkg

# Import the asclinic message types
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

rospack = rospkg.RosPack()
rospack.get_path('asclinic_pkg')

class GetPathCSV:

    def __init__(self):
        # # Initialise a publisher
        self.warehouse_map_publisher = rospy.Publisher("/WH_map_ROS", Polygon, queue_size=1)
        self.transit_path_publisher = rospy.Publisher("/transit_path_Matlab", Polygon, queue_size=1)
        self.planned_path_publisher = rospy.Publisher("/planned_path_Matlab", Polygon, queue_size=1)

        # # Initialise a subscriber
        # rospy.Subscriber(node_name+"/template_topic", UInt32, self.templateSubscriberCallback)
        rospy.Timer(rospy.Duration(1.0), self.readCSV_Map_Warehouse, oneshot = True)
        rospy.Timer(rospy.Duration(1.0), self.readCSV_transitPath, oneshot = True)
        rospy.Timer(rospy.Duration(1.0), self.readCSV_plannedPath, oneshot = True)



        self.warehouse_map = []
        self.counter = 0
        self.mapPolygon = Polygon()
        self.transitPathPolygon = Polygon()
        self.plannedPathPolygon = Polygon()

    def readCSV_Map_Warehouse(self,event):
        with open(rospack.get_path('asclinic_pkg')+'/src/nodes/warehouse_spec.csv','r') as csv_file:
            csv_reader = csv.DictReader(csv_file)

            for row in csv_reader:
                point32 = Point32()
                point32.x = float(row['x_axis'])
                point32.y = float(row['y_axis'])
                point32.z = 0
                self.mapPolygon.points.append(point32)
            # Now publish to a cpp node to store as global
            self.warehouse_map_publisher.publish(self.mapPolygon)
    
    # Function to read csv file from transit Path
    def readCSV_transitPath(self,event):
        with open(rospack.get_path('asclinic_pkg')+'/src/nodes/transitPath.csv','r') as csv_file:
            csv_reader = csv.DictReader(csv_file)

            for row in csv_reader:
                point32 = Point32()
                point32.x = float(row['x_axis'])
                point32.y = float(row['y_axis'])
                point32.z = float(row['heading'])
                self.transitPathPolygon.points.append(point32)
            # Now publish to a cpp node to store as global
            self.transit_path_publisher.publish(self.transitPathPolygon)

    # Function to read csv file from planned Path
    def readCSV_plannedPath(self,event):
        with open(rospack.get_path('asclinic_pkg')+'/src/nodes/plannedPath.csv','r') as csv_file:
            csv_reader = csv.DictReader(csv_file)

            for row in csv_reader:
                point32 = Point32()
                point32.x = float(row['x_axis'])
                point32.y = float(row['y_axis'])
                point32.z = float(row['heading'])
                self.plannedPathPolygon.points.append(point32)
            # Now publish to a cpp node to store as global
            self.planned_path_publisher.publish(self.plannedPathPolygon)


if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "publish_Path_ROS"
    rospy.init_node(node_name)
    getPath = GetPathCSV()
    # getPath.readCSV_Map_Warehouse()
    # getPath.readCSV_transitPath()
    # getPath.readCSV_plannedPath()
    # Spin as a single-threaded node
    rospy.spin()