#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import the ROS-Python package
import rospy

# Import the standard message types
from std_msgs.msg import UInt32

# Import the asclinic message types
from asclinic_pkg.msg import TransformationMatrix
from asclinic_pkg.msg import AmrPose

import numpy as np
import math


# Constants & Variables
homo_row = np.array([[0,0,0,1]])
h = 0.0 # Height of marker center

R_gm_04     = np.array([[1,0,0], [0,0,1], [0,-1,0]])
R_gm_59     = np.array([[0,0,1], [-1,0,0], [0,-1,0]])
R_gm_1015   = np.array([[-1,0,0], [0,0,-1], [0,-1,0]])
R_gm_1618   = np.array([[0,0,-1], [1,0,0], [0,-1,0]])
R_gm_19     = R_gm_04
R_gm_20     = R_gm_1618

t_gm_x = np.array([[2.0,1.6,1.2,0.8,0.4,    0.0,0.0,0.0,0.0,0.0,        0.4,0.8,1.2,1.6,2.0,3.0,        3.6,3.6,3.6,        3.0,    2.4]])
t_gm_y = np.array([[0.0,0.0,0.0,0.0,0.0,    0.58,1.18,1.78,2.38,2.98,   3.56,3.56,3.56,3.56,3.56,3.56,  2.98,2.38,1.78,     1.2,    0.58]])
t_gm_z = h*np.reshape(np.ones(21), (1,21))
t_gm_all = np.concatenate((t_gm_x, t_gm_y, t_gm_z), axis=0)


class PoseEstimationCam:

    def __init__(self):
        self.publisher_4_pose_estimation_cam = rospy.Publisher('estimated_pose_from_cam', AmrPose, queue_size=1)
        rospy.Subscriber('transformation_mat_cam_to_marker', TransformationMatrix, self.SubscriberCallback4TransformationMat)
        rospy.Subscriber('timer_4_camera_control', UInt32, self.SubscriberCallback4TimerCameraControl)

        self.T_cb = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, -0.035], [0, 0, 0, 1]])

        self.T_gm = np.expand_dims(self.gen_transform_matrix(rmat=R_gm_04, tvec=t_gm_all[:,[0]]), axis=0)
        for i in range(1, 21):
            if i <= 4:
                rmat = R_gm_04
            elif i <= 9:
                rmat = R_gm_59
            elif i <= 15:
                rmat = R_gm_1015
            elif i <= 18:
                rmat = R_gm_1618
            elif i <= 19:
                rmat = R_gm_19
            else:
                rmat = R_gm_20

            T_gm_i = np.expand_dims(self.gen_transform_matrix(rmat=rmat, tvec=t_gm_all[:, [i]]), axis=0)
            self.T_gm   = np.append(self.T_gm, T_gm_i, axis=0)



        self.id_counter = 0
        self.ave_pose_x = 0
        self.ave_pose_y = 0
        self.ave_pose_phi = 0

        self.is_pose_estimated = False

        self.default_pose_cam = AmrPose()
        self.default_pose_cam.pose_x = 10000
        self.default_pose_cam.pose_y = 10000
        self.default_pose_cam.pose_phi = 10000

    
    def gen_transformation_mat_Tgm(self):
        T_gm = np.expand_dims(self.gen_transform_matrix(rmat=R_gm_04, tvec=t_gm_all[:,[0]]), axis=0)
        for i in range(1, 21):
            if i <= 4:
                rmat = R_gm_04
            elif i <= 9:
                rmat = R_gm_59
            elif i <= 15:
                rmat = R_gm_1015
            elif i <= 18:
                rmat = R_gm_1618
            elif i <= 19:
                rmat = R_gm_19
            else:
                rmat = R_gm_20

            T_gm_i = np.expand_dims(self.gen_transform_matrix(rmat=rmat, tvec=t_gm_all[:, [i]]), axis=0)
            T_gm   = np.append(T_gm, T_gm_i, axis=0)

            return T_gm

    # Respond to subscriber receiving a message
    def SubscriberCallback4TransformationMat(self, msg):
        rmat = np.reshape(msg.rmat, (3,3))
        tvec = np.reshape(msg.tvec, (3,1))

        self.T_cm = self.gen_transform_matrix(rmat, tvec)
        self.T_mc = np.linalg.inv(self.T_cm)

        self.current_pose_cam = self.estimate_pose(msg.id)

        if (msg.num_aruco_ids_detect >= 1) and (self.id_counter<=msg.num_aruco_ids_detect) :
            self.id_counter = self.id_counter + 1
            
            self.ave_pose_x     = (self.ave_pose_x * (self.id_counter-1) + self.current_pose_cam.pose_x)/(self.id_counter)
            self.ave_pose_y     = (self.ave_pose_y * (self.id_counter-1) + self.current_pose_cam.pose_y)/(self.id_counter)
            self.ave_pose_phi   = (self.ave_pose_phi * (self.id_counter-1) + self.current_pose_cam.pose_phi)/(self.id_counter)

            if (self.id_counter == msg.num_aruco_ids_detect):
                self.current_pose_cam.pose_x     = self.ave_pose_x 
                self.current_pose_cam.pose_y     = self.ave_pose_y 
                self.current_pose_cam.pose_phi   = self.ave_pose_phi 
                # self.publisher_4_pose_estimation_cam.publish(self.current_pose_cam)

                # rospy.loginfo("Pose x: " + str(self.current_pose_cam.pose_x) + " Pose y: " + str(self.current_pose_cam.pose_y) + " Heading: " + str(self.current_pose_cam.pose_phi) )
                #rospy.loginfo(" Heading: " + str(self.current_pose_cam.pose_phi) )    
                self.id_counter = 0
                self.ave_pose_x = 0
                self.ave_pose_y = 0
                self.ave_pose_phi = 0

                self.is_pose_estimated = True


    def estimate_pose(self, id):
        pose = AmrPose()
        T_gm_i = self.T_gm[id]
        self.T_gb = np.matmul(self.T_gm[id], np.matmul(self.T_mc, self.T_cb))
        pose.pose_x = self.T_gb[0,3]
        pose.pose_y = self.T_gb[1,3]

        cos_theta       = math.sqrt(self.T_gb[0,1]*self.T_gb[0,1] + self.T_gb[0,0]*self.T_gb[0,0])
        pose.pose_phi   = -math.atan2(self.T_gb[0,1]/cos_theta, self.T_gb[0,0]/cos_theta)

        #rospy.loginfo("T_gm_i: " + str(T_gm_i[0][0]) + " / " + str(T_gm_i[0][1]) + " / " + str(T_gm_i[0][2]) + " /// " + str(T_gm_i[1][0]) + " / " + str(T_gm_i[1][1]) + " / " + str(T_gm_i[1][2]) + " /// " + str(T_gm_i[2][0]) + " / " + str(T_gm_i[2][1]) + " / " + str(T_gm_i[2][2]))  

        #rospy.loginfo("T_gb: " + str(self.T_gb[0][0]) + " / " + str(self.T_gb[0][1]) + " / " + str(self.T_gb[0][2]) + " /// " + str(self.T_gb[1][0]) + " / " + str(self.T_gb[1][1]) + " / " + str(self.T_gb[1][2]) + " /// " + str(self.T_gb[2][0]) + " / " + str(self.T_gb[2][1]) + " / " + str(self.T_gb[2][2]))  
        return pose


    def gen_transform_matrix(self, rmat, tvec):
        mat = np.concatenate((rmat, tvec), axis=1)
        return np.concatenate((mat, homo_row), axis=0)
        

    def SubscriberCallback4TimerCameraControl(self, event):
        published_pose = AmrPose()

        if (self.is_pose_estimated):
            published_pose = self.current_pose_cam
            self.is_pose_estimated = False
        else:
            published_pose = self.default_pose_cam

        self.publisher_4_pose_estimation_cam.publish(published_pose)
        # rospy.loginfo("Pose x: " + str(published_pose.pose_x) + " Pose y: " + str(published_pose.pose_y) + " Heading: " + str(published_pose.pose_phi) )


if __name__ == '__main__':
    global node_name
    node_name = "pose_estimator_cam"

    rospy.init_node(node_name)
    node = PoseEstimationCam()
    
    rospy.spin()
