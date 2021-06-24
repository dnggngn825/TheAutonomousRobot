#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# Import the ROS-Python package
import rospy
#import rospkg

# Import the standard message types
from std_msgs.msg import UInt32
from sensor_msgs.msg import Image
from asclinic_pkg.msg import TransformationMatrix

import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge


# DEFINE THE PARAMETERS
USB_CAMERA_DEVICE_NUMBER = 0    # > For the number of the USB camera device. for /dev/video0, this parameter should be 0
#MARKER_SIZE = 0.2130                # Lab Tests
#MARKER_SIZE = 0.145                  # Home Tests
MARKER_SIZE = 0.151                 # MPL 
TIMER_DURATION = 0.064



class ArucoDetector:

    def __init__(self):
        self.publisher_4_transformation_mat = rospy.Publisher('transformation_mat_cam_to_marker', TransformationMatrix, queue_size=10)


        self.camera_setup   = USB_CAMERA_DEVICE_NUMBER       
        self.cam            = cv2.VideoCapture(self.camera_setup) # Initialise video capture from the camera       
        self.cv_bridge      = CvBridge() # Initlaise the OpenCV<->ROS bridge


        self.aruco_dict         = aruco.Dictionary_get(aruco.DICT_4X4_50) # Get the ArUco dictionary to use
        self.aruco_parameters   = aruco.DetectorParameters_create() # Create an parameter structure needed for the ArUco detection
        self.aruco_parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX # > Specify the parameter for: corner refinement


        self.intrinic_camera_matrix     = np.array( [[639.61935325,0,313.65162393] , [0,640.25521442,243.24068703] , [0,0,1]], dtype=float)
        self.intrinic_camera_distortion = np.array( [[0.05328607,-0.16649145,-0.00126354,-0.00216118,0.06680599]], dtype=float)


        rospy.loginfo("[ARUCO] Initialised")
        rospy.Timer(rospy.Duration(TIMER_DURATION), self.timerCallbackForPublishing)
        

    def timerCallbackForPublishing(self, event):
        return_flag , current_frame = self.cam.read()

        if (return_flag == True): # Check if the camera frame was successfully read
            current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers from the frame
            aruco_corners_of_all_markers, aruco_ids, aruco_rejected_img_points = aruco.detectMarkers(current_frame_gray, self.aruco_dict, parameters=self.aruco_parameters)

            # Process any ArUco markers that were detected
            if aruco_ids is not None:
                num_aruco_ids_detect = len(aruco_ids)
                #rospy.loginfo("[CAMERA_MONITOR] ArUco marker found, len(aruco_ids) = " + str(len(aruco_ids)))
                
                # Iterate over the markers detected
                for i_marker_id in range(num_aruco_ids_detect):
                    this_id                 = aruco_ids[i_marker_id]
                    corners_of_this_marker  = aruco_corners_of_all_markers[i_marker_id]
                    
                    # Estimate the pose of this marker
                    this_rvec_estimate, this_tvec_estimate, _objPoints = aruco.estimatePoseSingleMarkers(corners_of_this_marker, MARKER_SIZE, self.intrinic_camera_matrix, self.intrinic_camera_distortion)
                    
                    # Draw the pose of this marker
                    rvec = this_rvec_estimate[0]
                    tvec = this_tvec_estimate[0]
                    # current_frame_with_marker_outlines = aruco.drawAxis(current_frame_with_marker_outlines, self.intrinic_camera_matrix, self.intrinic_camera_distortion, rvec, tvec, MARKER_SIZE)
                    rvec = rvec[0]
                    tvec = tvec[0]
                    
                    # Compute the rotation matrix from the rvec using the Rodrigues
                    Rmat = cv2.Rodrigues(rvec)
                    Rmat = Rmat[0]
                    #rospy.loginfo("Rotational Matrix:")
                    #rospy.loginfo(str(Rmat[0][0]) + " / " + str(Rmat[0][1])+ " / " + str(Rmat[0][2]))
                    #rospy.loginfo(str(Rmat[1][0]) + " / " + str(Rmat[1][1])+ " / " + str(Rmat[1][2]))
                    #rospy.loginfo(str(Rmat[2][0]) + " / " + str(Rmat[2][1])+ " / " + str(Rmat[2][2]))

                    # ============================================
                    # TO BE FILLED IN FOR WORLD FRAME LOCALISATION
                    # ============================================
                    t_matrix        = TransformationMatrix()
                    t_matrix.rmat   = np.reshape(Rmat, (1,9))[0].tolist()
                    t_matrix.tvec   = np.reshape(tvec, (1,3))[0].tolist()
                    t_matrix.id     = int(this_id[0])
                    t_matrix.num_aruco_ids_detect = int(num_aruco_ids_detect)
                    self.publisher_4_transformation_mat.publish(t_matrix)
                    #rospy.loginfo("[CAMERA_MONITOR]  #" + str(len(aruco_ids)) + " markers found with IDs:  " + str(this_id))

            #else:
                # Display that no aruco markers were found
                # rospy.loginfo("[CAMERA_MONITOR] No markers found in this image")


        else:
            # Display an error message
            rospy.loginfo('[CAMERA_MONITOR] ERROR occurred during self.cam.read()')


if __name__ == '__main__':
    global node_name
    node_name = "camera_monitor"
    
    rospy.init_node(node_name)
    camera_monitor = ArucoDetector()

    rospy.spin()

    camera_monitor.cam.release()
    cv2.destroyAllWindows()
