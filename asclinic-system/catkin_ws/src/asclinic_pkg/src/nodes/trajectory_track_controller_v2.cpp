/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Hai Dang Nguyen (860308)
================================================== */

/**
 * @brief The node subscribes to get traj ref signal from motion planning
 * control path following and purely rotation and publish the desired robot movement/command
 * to the internal controller of the robot.
 * 
 */

#include "ros/ros.h"
#include <math.h>
#include <stdlib.h>
#include "amr/amr.h"
#include "amr/amr_constants.h"

#include "asclinic_pkg/Polygon.h"
#include "asclinic_pkg/Point32.h"
#include "asclinic_pkg/AmrPose.h"
#include "asclinic_pkg/MotorAngularVelocity.h"
#include "MotionPlanHeader.h"
#include "std_msgs/Bool.h"

using namespace asclinic_pkg;



// Init global variables
AmrPose pose_current = AmrPose();
AmrPose pose_ref     = AmrPose();
MotorAngularVelocity angular_velocity_msg;

discreteParam error_heading;
discreteParam error_location;
discreteParam v;
discreteParam w;
discreteParam angular_velocity_l;
discreteParam angular_velocity_r;
discreteParam heading_ref;

bool current_movement_type = false;
bool in_movement = false;


ros::Publisher publisher_4_internal_control;


void subscriberCallback4PoseReference(const AmrPose& msg);
void subscriberCallback4PoseEstimated(const AmrPose& msg);
void subscriberCallback4MovementType(const std_msgs::Bool& msg);
void subscriberCallback4InMovement(const std_msgs::Bool& msg);
// void subscriberCallback4DynamicState(const std_msgs::Bool& msg);

void updateRefPose(const AmrPose& msg);
float PController4Heading(discreteParam error);
float PController4PathFollowing(discreteParam error);
float PController4HeadingLinear(discreteParam error_heading);
float withInPi(float angle);


int main(int argc, char* argv[])
{
    ros::init(argc, argv, amr_node::TRAJ_TRACK_CONTROLLER);
    ros::NodeHandle nodeHandle;

    ros::Subscriber subscribe_4_pose_est        = nodeHandle.subscribe(amr_topic::POSE_EST, 1, subscriberCallback4PoseEstimated);
	ros::Subscriber subscribe_4_pose_ref        = nodeHandle.subscribe(amr_topic::TRAJECTORY_REF_SIGNAL, 1, subscriberCallback4PoseReference);
    ros::Subscriber subscribe_4_movement_type   = nodeHandle.subscribe(amr_topic::MOVEMENT_TYPE, 1, subscriberCallback4MovementType);
    ros::Subscriber subscribe_4_in_movement     = nodeHandle.subscribe("in_movement", 1, subscriberCallback4InMovement);

    publisher_4_internal_control = nodeHandle.advertise<MotorAngularVelocity>(amr_topic::MOTOR_ANGULAR_VELOCITY_REF, 1, false);

    pose_current.pose_x = 0.0; pose_current.pose_y = 0.0; pose_current.pose_phi = 0.0;
    pose_ref.pose_x = 0.0; pose_ref.pose_y = 0.0; pose_ref.pose_phi = 0.0;


    ros::spin();

	return 0;
}


/**
 * @brief compute the error and feed into PI controller to get v and omega
 * 
 * @param msg reference pose of the robot obtained from motion planning
 */
void subscriberCallback4PoseReference(const AmrPose& msg)
{
    pose_ref = msg;
    ROS_INFO_STREAM("Current REF pose: "<< pose_ref.pose_x<< "; "<<pose_ref.pose_y<< "; "<<pose_ref.pose_phi);

    heading_ref.current = pose_ref.pose_phi;
    heading_ref.updateNewValue();
}

/**
 * @brief update parameter pose_current of the robot obtained from odometry est
 * 
 * @param msg 
 */

void subscriberCallback4PoseEstimated(const AmrPose& msg)
{
    // ROS_INFO_STREAM("IN MOVEMNET IN CONTROL "<< in_movement);
    
    if (in_movement)
    {
        pose_current = msg;
        ROS_INFO_STREAM("Current pose: "<< pose_current.pose_x<< "; "<<pose_current.pose_y<< "; "<<pose_current.pose_phi);

        if (current_movement_type)
        {
            // Rotation
            error_location.current = 0;
            error_heading.current = withInPi((pose_ref.pose_phi) - (pose_current.pose_phi));
            ROS_INFO_STREAM(" ---- Rotation --- HEADING ERROR:  " << error_heading.current);
        }
        else
        {
            // Linear
            error_location.current = sqrt(pow((pose_ref.pose_x - pose_current.pose_x),2)+pow((pose_ref.pose_y - pose_current.pose_y),2));
            float deltaY = pose_ref.pose_y - pose_current.pose_y;
            float deltaX = pose_ref.pose_x - pose_current.pose_x;
            error_heading.current = withInPi(atan2(deltaY,deltaX) - (pose_current.pose_phi));
            ROS_INFO_STREAM("--- Heading ref: "<< round(atan2(deltaY,deltaX)*100.0)/100.0 << std::endl);
            ROS_INFO_STREAM(" ---- Linear --- HEADING ERROR:  " << error_heading.current);
        }
        
        // pure rotation movement
        if (current_movement_type)
        {
            v.current = PController4PathFollowing(error_location)*1.15;
            w.current = PController4Heading(error_heading)*1.15;
        }
        // Linear movement
        else
        {
            v.current = PController4PathFollowing(error_location);
            w.current = PController4HeadingLinear(error_heading);
        }

        angular_velocity_l.current = (v.current - AMR_BASE_HALF_WHEEL_BASE*w.current)/AMR_BASE_RADIUS_WHEEL_L;
        angular_velocity_r.current = (v.current + AMR_BASE_HALF_WHEEL_BASE*w.current)/AMR_BASE_RADIUS_WHEEL_R;

        ROS_INFO_STREAM("[TRAJ TRACK ANG] ref angular vel current (L/ R)"<< angular_velocity_l.current << "; "<<angular_velocity_r.current);

        angular_velocity_msg.angular_velocity_motor_l = angular_velocity_l.current;
        angular_velocity_msg.angular_velocity_motor_r = angular_velocity_r.current;

        publisher_4_internal_control.publish(angular_velocity_msg);


        // Save pre values
        error_heading.updateNewValue();
        error_location.updateNewValue();

        v.updateNewValue();
        w.updateNewValue();

        angular_velocity_l.updateNewValue();
        angular_velocity_r.updateNewValue();
    }
    else 
    {
        angular_velocity_msg.angular_velocity_motor_l = 0;
        angular_velocity_msg.angular_velocity_motor_r = 0;

        publisher_4_internal_control.publish(angular_velocity_msg);
    }
}

void subscriberCallback4MovementType(const std_msgs::Bool& msg)
{
    current_movement_type = msg.data;
    // in_movement = 1;
    if (current_movement_type)
        ROS_INFO_STREAM("[MOVEMENT]: -- ROTATION --");
    else
        ROS_INFO_STREAM("[MOVEMENT]: -- LINEAR --" );
}

void subscriberCallback4InMovement(const std_msgs::Bool& msg)
{
    in_movement = msg.data;
    // ROS_INFO_STREAM("IN MOVEMENT " << in_movement);
    
}

/**
 * @brief PI controller for heading and path following
 * 
 * @param error_heading
 * @return float W from heading and V from pathFollowing
 */
float PController4Heading(discreteParam error_heading)
{
    PIDController headingController = PIDController(0.8,0,0);
    return headingController.discreteP(error_heading);
}

float PController4HeadingLinear(discreteParam error_heading)
{
    PIDController headingController = PIDController(0.8,0,0);
    return headingController.discreteP(error_heading);
}

float PController4PathFollowing(discreteParam error_location)
{
    PIDController pathFollowController = PIDController(0.6,0.0,0.0);
    return pathFollowController.discreteP(error_location);
}

float withInPi(float angle)
{
    while (angle >= M_PI)
    {
        angle = angle - M_PI*2;
    }

    if ((angle > 0) and (angle > M_PI))
    {
        angle = angle - M_PI*2;
    }
    else if ((angle < 0) && (angle < -M_PI))
    {
        angle  = angle + M_PI*2;

    }
    return angle;
}