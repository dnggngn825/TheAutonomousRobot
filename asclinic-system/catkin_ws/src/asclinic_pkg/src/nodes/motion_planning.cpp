/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Hai Dang Nguyen (860308)
================================================== */

/**
 * @brief The node subscribes to get planned path from matlab, make decision and 
 * publish the reference node to the trajectory tracking controller.
 * 
 */

#include "ros/ros.h"
#include "amr/amr.h"
#include <math.h>
#include <stdlib.h>

#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "asclinic_pkg/AmrPose.h"
#include "std_msgs/Float64.h"

using namespace asclinic_pkg;

// geometry_msgs::Polygon transitPath = geometry_msgs::Polygon();
// geometry_msgs::Polygon plannedPath = geometry_msgs::Polygon();
// bool transit = false;
// bool planned = false;

struct pathStruct
{
    geometry_msgs::Polygon path = geometry_msgs::Polygon();
    bool state = false;
};
pathStruct transitPath = pathStruct();
pathStruct plannedPath = pathStruct();

ros::Publisher publisher_4_trajectory_tracking;
ros::Publisher publisher_state_changePath;
ros::Publisher publisher_4_testing_Matlab;

void subscriberCallback4TransitPath(const geometry_msgs::Polygon& msg);
void subscriberCallback4PlannedPath(const geometry_msgs::Polygon& msg);
void testingMatlab(const std_msgs::Float64& msg);


int main(int argc, char* argv[])
{
    ros::init(argc, argv, amr_node::MOTION_PLANNING);
    ros::NodeHandle nodeHandle;

	ros::Subscriber subscribe_transit_path = nodeHandle.subscribe(amr_topic::TRANSIT_PATH_MATLAB, 1, subscriberCallback4TransitPath);
    ros::Subscriber subscribe_planned_path = nodeHandle.subscribe(amr_topic::PLANNED_PATH_MATLAB, 1, subscriberCallback4PlannedPath);

    // publisher_4_trajectory_tracking = nodeHandle.advertise<AmrPose>(amr_topic::TRAJECTORY_REF_SIGNAL, 1, false);
    // publisher_4_testing_Matlab = nodeHandle.advertise<geometry_msgs::Polygon>(amr_topic::TRANSIT_PATH_MATLAB, 1, false);

    
    // need to convert point 32 to AmrPose
    ros::Rate rate(10);
    

    // while (ros::ok())
    // {
    //     // insert the code
    //     geometry_msgs::Polygon msg2;
    //     geometry_msgs::Point32 point32;
    //     point32.x = 1.1;
    //     point32.y = 2.2;
    //     point32.z = 3.3;
    //     msg2.points.push_back(point32);
    //     publisher_4_testing_Matlab.publish(msg2);

    //     ros::spinOnce();
    //     rate.sleep();
        
    // }
    while (ros::ok())
    {
        if (transitPath.state)
        {
            ROS_INFO_STREAM(transitPath.path.points[1].x << "; "<< transitPath.path.points[1].y);
            transitPath.state = false;
            ROS_INFO_STREAM(plannedPath.path.points[1].x << "; "<< plannedPath.path.points[1].y);
        }

        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}

void testingMatlab(const std_msgs::Float64& msg)
{
    ROS_INFO_STREAM(msg.data << "Hello world " << std::endl);
}

void subscriberCallback4TransitPath(const geometry_msgs::Polygon& msg)
{
    ROS_INFO_STREAM("HELLO WORLD transit Path: ================ " << std::endl);
    transitPath.path = msg;
    transitPath.state = true;
    
}

void subscriberCallback4PlannedPath(const geometry_msgs::Polygon& msg)
{
    ROS_INFO_STREAM("HELLO WORLD planned Path: " << std::endl);
    plannedPath.path = msg;
    plannedPath.state = true;
}