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
// AmrPose currentPose = AmrPose();
// AmrPose refPose = AmrPose();
// MotorAngularVelocity RotationAngleMsg;
discreteParam headingError;
discreteParam pathError;
discreteParam v;
discreteParam w;
discreteParam velocity_wheel_l;
discreteParam velocity_wheel_r;
discreteParam refHeading;
bool current_movement_type;
// bool linear = 0;
bool in_movement = false;


ros::Publisher publisher_4_internal_control;


void subscriberCallback4TrajRefSignal(const AmrPose& msg);
void subscriberCallback4OdoFeedback(const AmrPose& msg);
void subscriberCallback4MovementType(const std_msgs::Bool& msg);
void subscriberCallback4InMovement(const std_msgs::Bool& msg);
// void subscriberCallback4DynamicState(const std_msgs::Bool& msg);

void updateRefPose(const AmrPose& msg);
float PIController4Heading(discreteParam error);
float PIController4PathFollowing(discreteParam error);
float PIController4HeadingLinear(discreteParam headingError);
float withInPi(float angle);


int main(int argc, char* argv[])
{
    ros::init(argc, argv, amr_node::TRAJ_TRACK_CONTROLLER);
    ros::NodeHandle nodeHandle;

    ros::Subscriber subscribe_odo_pose_feedback = nodeHandle.subscribe(amr_topic::POSE_EST, 1, subscriberCallback4OdoFeedback);
	ros::Subscriber subscribe_traj_ref_signal = nodeHandle.subscribe(amr_topic::TRAJECTORY_REF_SIGNAL, 1, subscriberCallback4TrajRefSignal);
    ros::Subscriber subscribe_4_movement_type = nodeHandle.subscribe(amr_topic::MOVEMENT_TYPE, 1, subscriberCallback4MovementType);
    // ros::Subscriber subscribe_4_dynamic_state = nodeHandle.subscribe("dynamic_state", 1, subscriberCallback4DynamicState);
    ros::Subscriber subscribe_4_in_movement = nodeHandle.subscribe("in_movement", 1, subscriberCallback4InMovement);

    publisher_4_internal_control = nodeHandle.advertise<MotorAngularVelocity>(amr_topic::MOTOR_ANGULAR_VELOCITY_REF, 1, false);

    ros::Rate rate(10);

    currentPose.pose_x = 0.0; currentPose.pose_y = 0.0; currentPose.pose_phi = 0.0;
    refPose.pose_x = 0.0; refPose.pose_y = 0.0; refPose.pose_phi = 0.0;
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    

	return 0;
}


/**
 * @brief compute the error and feed into PI controller to get v and omega
 * 
 * @param msg reference pose of the robot obtained from motion planning
 */
void subscriberCallback4TrajRefSignal(const AmrPose& msg)
{
    ROS_INFO_STREAM("HELLO WORLD");
    updateRefPose(msg);
}

/**
 * @brief update parameter currentPose of the robot obtained from odometry est
 * 
 * @param msg 
 */

void subscriberCallback4OdoFeedback(const AmrPose& msg)
{
    ROS_INFO_STREAM("IN MOVEMNET IN CONTROL "<< in_movement);
    
    if (in_movement)
    {
        currentPose = msg;

        ROS_INFO_STREAM("Current pose: "<< currentPose.pose_x<< "; "<<currentPose.pose_y<< "; "<<currentPose.pose_phi);


        refHeading.updateNewValue();
        refHeading.current = refPose.pose_phi;
        
        headingError.updateNewValue();
        pathError.updateNewValue();

        if (current_movement_type)
        {
            // Rotation
            pathError.current = 0;
            headingError.current = withInPi((refPose.pose_phi) - (currentPose.pose_phi));
            ROS_INFO_STREAM(" ---- Rotation --- HEADING ERROR:  " << headingError.current);
        }
        else
        {
            // Linear
            pathError.current = sqrt(pow((refPose.pose_x - currentPose.pose_x),2)+pow((refPose.pose_y - currentPose.pose_y),2));
            float deltaY = refPose.pose_y - currentPose.pose_y;
            float deltaX = refPose.pose_x - currentPose.pose_x;
            headingError.current = withInPi(atan2(deltaY,deltaX) - (currentPose.pose_phi));
            ROS_INFO_STREAM("--- Heading ref: "<< round(atan2(deltaY,deltaX)*100.0)/100.0 << std::endl);
            ROS_INFO_STREAM(" ---- Linear --- HEADING ERROR:  " << headingError.current);
            // headingError.current = 0;
        }

        v.updateNewValue();
        w.updateNewValue();
        
        // pure rotation movement
        if (current_movement_type)
        {
            v.current = PIController4PathFollowing(pathError);
            w.current = PIController4Heading(headingError);
            ROS_INFO_STREAM("[ROTATION] VELOCITY & W " << v.current << ", " << w.current);
        }
        // Linear movement
        else
        {
            v.current = PIController4PathFollowing(pathError);
            w.current = PIController4HeadingLinear(headingError);
            ROS_INFO_STREAM("[LINEAR] VELOCITY & W " << v.current << ", " << w.current);

        }
        

        velocity_wheel_l.updateNewValue();
        velocity_wheel_r.updateNewValue();

        // if (refPose.pose_y >= 0.6)
        // {
        //     velocity_wheel_l.current = 0;
        //     velocity_wheel_r.current = 0;
        // }
        // else
        // {
            velocity_wheel_l.current = (v.current - AMR_BASE_HALF_WHEEL_BASE*w.current)/AMR_BASE_RADIUS_WHEEL_L;
            velocity_wheel_r.current = (v.current + AMR_BASE_HALF_WHEEL_BASE*w.current)/AMR_BASE_RADIUS_WHEEL_R;
        // }

        RotationAngleMsg.angular_velocity_motor_l = velocity_wheel_l.current;
        RotationAngleMsg.angular_velocity_motor_r = velocity_wheel_r.current;

        // RotationAngleMsg.angular_velocity_motor_l = -2;
        // RotationAngleMsg.angular_velocity_motor_r = 2;

        ROS_INFO_STREAM("[TRAJ TRACK ANG] ref angular vel current (L/ R)"<< velocity_wheel_l.current << "; "<<velocity_wheel_r.current);
        ROS_INFO_STREAM("[TRAJ TRACK ANG] ref angular vel previous (L/ R)"<< velocity_wheel_l.prev << "; "<<velocity_wheel_r.prev);

        
            publisher_4_internal_control.publish(RotationAngleMsg);
        }
        else 
        {
            RotationAngleMsg.angular_velocity_motor_l = 0;
            RotationAngleMsg.angular_velocity_motor_r = 0;

            publisher_4_internal_control.publish(RotationAngleMsg);
        }
// <<<<<<< HEAD
//     // else
//     // {
//     //     RotationAngleMsg.angular_velocity_motor_l = 0;
//     //     RotationAngleMsg.angular_velocity_motor_r = 0;
//     //     publisher_4_internal_control.publish(RotationAngleMsg);
//     // }
// =======
//     else
//     {
//         RotationAngleMsg.angular_velocity_motor_l = 0;
//         RotationAngleMsg.angular_velocity_motor_r = 0;
//         publisher_4_internal_control.publish(RotationAngleMsg);
//     }
// >>>>>>> 0d8ea72621c4708be6b367494de3b37fc27a6103
}

void updateRefPose(const AmrPose& msg)
{
    refPose = msg;
    ROS_INFO_STREAM("Current REF pose: "<< refPose.pose_x<< "; "<<refPose.pose_y<< "; "<<refPose.pose_phi);
}

// void subscriberCallback4DynamicState(const std_msgs::Bool& msg)
// {
//     dynamic_state = msg.data;
// }

void subscriberCallback4MovementType(const std_msgs::Bool& msg)
{
    current_movement_type = msg.data;
    // in_movement = 1;
    if (msg.data)
    {
        ROS_INFO_STREAM("[MOVEMENT]: -- ROTATION --");
    }
    else
    {
        ROS_INFO_STREAM("[MOVEMENT]: -- LINEAR --" );
    }
}

void subscriberCallback4InMovement(const std_msgs::Bool& msg)
{
    in_movement = msg.data;
    ROS_INFO_STREAM("IN MOVEMENT " << in_movement);
    
}

/**
 * @brief PI controller for heading and path following
 * 
 * @param headingError
 * @return float W from heading and V from pathFollowing
 */
float PIController4Heading(discreteParam headingError)
{
    PIDController headingController = PIDController(1,0,0);

    // PIDController headingController = PIDController(7.0,0.1,0.0);
    return headingController.discreteP(headingError);
}

float PIController4HeadingLinear(discreteParam headingError)
{
    PIDController headingController = PIDController(2.5,0,0);
    // PIDController headingController = PIDController(0.5,0,0);

    // PIDController headingController = PIDController(7.0,0.1,0.0);
    return headingController.discreteP(headingError);
}

float PIController4PathFollowing(discreteParam pathError)
{
    PIDController pathFollowController = PIDController(0.8,0.0,0.0);
    return pathFollowController.discreteP(pathError);
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