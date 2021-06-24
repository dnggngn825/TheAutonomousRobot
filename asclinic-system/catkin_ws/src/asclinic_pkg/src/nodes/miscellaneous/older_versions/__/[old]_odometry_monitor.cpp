/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


#include "ros/ros.h"
#include "std_msgs/String.h"

// Include the asclinic message types
#include "asclinic_pkg/PoseData.h"

// Namespacing the package
using namespace asclinic_pkg;




int main(int argc, char* argv[]){
    ros::init(argc, argv, "odometry_monitor"); // init ROS, allow ROS to remao the node
    ros::NodeHandle nodeHandle; // create a handle
    // ros::subscriber sub = nh.subscribe("odometry", 10, OdometryCallback); // subcribe to odometry topic. ROS calls odometryCallback() when new message arrives.
    
    // ros::Publisher raw_pose = nodeHandle.advertise<std_msgs::String>("raw_pose", 1000);
    ros::Publisher raw_pose = nodeHandle.advertise<PoseData>("raw_pose", 1000);
    ros::Rate loop_rate(1);

    while (ros::ok())
	{
        // std_msgs::String msg;
        // msg.data = "msg data";
        PoseData msg;
        msg.pose_x = 1.0;
        msg.pose_y = 2.0;
        msg.phi = 3.0;

        raw_pose.publish(msg);
        ROS_INFO_STREAM("svj: "<<msg.pose_x);

		// Spin once so that this node can service any
		// callbacks that this node has queued.
		ros::spinOnce();

		// Sleep for the specified loop rate
		loop_rate.sleep();
	} // END OF: "while (ros::ok())"

    // ROS_INFO("Hello, World!");

    return 0;
}
