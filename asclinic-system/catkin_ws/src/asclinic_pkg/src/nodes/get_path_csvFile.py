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
from asclinic_pkg.msg import pathCommand
from std_msgs.msg import UInt32

rospack = rospkg.RosPack()
rospack.get_path('asclinic_pkg')

class GetPathCSV:

    def __init__(self):
        # # Initialise a publisher
        self.transit_path_publisher     = rospy.Publisher("/transit_path_Matlab", Polygon, queue_size=1)
        self.planned_path_publisher     = rospy.Publisher("/planned_path_Matlab", Polygon, queue_size=1)
        self.test_path_publisher        = rospy.Publisher("/test_path", Polygon, queue_size=1)
        self.back2start_path_publisher  = rospy.Publisher("/back2start_path", Polygon, queue_size=1)
        self.notify_available_cycle     = rospy.Publisher("/notify_available_cycle", UInt32, queue_size = 1)

        rospy.Subscriber("/get_directory", pathCommand, self.notifyAvailableCycle)
        rospy.Subscriber("/get_next_cycle", UInt32, self.callback4ProceedNextCycle)

        self.counter                    = 0
        self.nextCycle                  = 0
        self.currentCycle               = -1
        self.cycleList                  = []
        self.mapPolygon                 = Polygon()
        self.transitPathPolygon         = Polygon()
        self.plannedPathPolygon         = Polygon()
        self.testPathPolygon            = Polygon()
        self.back2startPolygon          = Polygon()
        

        self.testPathDIR                = ""
        self.transitPathDIR             = ""
        self.plannedPathDIR             = ""
        self.back2startDIR              = ""
            


    # Function to read csv file from transit Path
    def readCSV_transitPath(self):
        with open(self.transitPathDIR,'r') as csv_file:
            csv_reader      = csv.DictReader(csv_file)

            for row in csv_reader:
                point32     = Point32()
                point32.x   = float(row['x_axis'])
                point32.y   = float(row['y_axis'])
                point32.z   = round(float(row['heading']),2)
                self.transitPathPolygon.points.append(point32)
            # Now publish to a cpp node to store as global
            # self.transit_path_publisher.publish(self.transitPathPolygon)

    # Function to read csv file from planned Path
    def readCSV_plannedPath(self):
        with open(self.plannedPathDIR,'r') as csv_file:
            csv_reader      = csv.DictReader(csv_file)

            for row in csv_reader:
                point32     = Point32()
                point32.x   = float(row['x_axis'])
                point32.y   = float(row['y_axis'])
                point32.z   = round(float(row['heading']),2)
                self.plannedPathPolygon.points.append(point32)
            # Now publish to a cpp node to store as global
            # self.planned_path_publisher.publish(self.plannedPathPolygon)

    # Function to read csv file from test Path
    def readCSV_testPath(self):
        with open(self.testPathDIR,'r') as csv_file:
            csv_reader      = csv.DictReader(csv_file)

            for row in csv_reader:
                point32     = Point32()
                point32.x   = float(row['x_axis'])
                point32.y   = float(row['y_axis'])
                point32.z   = round(float(row['heading']),2)
                self.testPathPolygon.points.append(point32)
            # Now publish to a cpp node to store as global
            # self.test_path_publisher.publish(self.testPathPolygon)

    def readCSV_back2StartPath(self):
        with open(self.back2startDIR,'r') as csv_file:
            csv_reader      = csv.DictReader(csv_file)

            for row in csv_reader:
                point32     = Point32()
                point32.x   = float(row['x_axis'])
                point32.y   = float(row['y_axis'])
                point32.z   = round(float(row['heading']),2)
                self.back2startPolygon.points.append(point32)

    def notifyAvailableCycle(self,msg):      
        # Get the list of request from user
        self.counter = len(msg.PathCommandList)
        self.cycleList = msg.PathCommandList
        self.notify_available_cycle.publish(self.counter)


    def getDir(self):
        stringDIR = rospack.get_path('asclinic_pkg')+ "/src/drivers/src/wh_trajectories/"
        stringDIR = rospack.get_path('asclinic_pkg')+ "/src/drivers/src/wh_trajectories copy/"
        self.transitPathDIR             = stringDIR + "S0_D" + str(self.cycleList[0]//10) + ".csv"
        self.plannedPathDIR             = stringDIR + "D" + str(self.cycleList[0]//10) + "_R"+ str(self.cycleList[0]%10) + ".csv"
        self.back2startDIR              = stringDIR + "R" + str(self.cycleList[0]%10) + "_S0.csv"
        # self.testPathDIR                = rospack.get_path('asclinic_pkg') + "/src/nodes/squarePath.csv"
        self.testPathDIR                = stringDIR + "S0_D1.csv"
        self.readCSV_transitPath()
        self.readCSV_back2StartPath()
        self.readCSV_plannedPath()
        self.readCSV_testPath()

    def processListofPath(self):
        # Publish the path to motion_planning
        self.test_path_publisher.publish(self.testPathPolygon)
        self.planned_path_publisher.publish(self.plannedPathPolygon)
        self.transit_path_publisher.publish(self.transitPathPolygon)
        self.transit_path_publisher.publish(self.transitPathPolygon)
        self.back2start_path_publisher.publish(self.back2startPolygon)
        print("number of request: "+ str(self.counter))

    def callback4ProceedNextCycle(self,msg):

        # receive message from idle state to process the list of require path
        if ((self.counter != 0) and (msg.data)):
            self.currentCycle += msg.data

            # update directory for each path
            self.getDir()
            self.processListofPath()
            print("[PATH PY.] Process" + str(self.counter) + " cycles at the moment\n")
            self.counter -= 1
            print("[PATH PY.] There is " + str(self.counter) + " cycles left\n")

    
    # def readPath(self,pub,dir):
    #     polygon = Polygon()
    #     with open(dir,'r') as csv_file:
    #         csv_reader      = csv.DictReader(csv_file)

    #         for row in csv_reader:
    #             point32     = Point32()
    #             point32.x   = float(row['x_axis'])
    #             point32.y   = float(row['y_axis'])
    #             point32.z   = round(float(row['heading']),2)
    #             polygon.points.append(point32)
    #         # Now publish to a cpp node to store as global
    #         pub.publish(polygon)


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

    # pathCommandList_msg = pathCommand()
    # pathCommandList_msg.PathCommandList = [11]
    # getPath.notifyAvailableCycle(pathCommandList_msg)
    # getPath.getDir()
    # getPath.processListofPath()

    rospy.spin()