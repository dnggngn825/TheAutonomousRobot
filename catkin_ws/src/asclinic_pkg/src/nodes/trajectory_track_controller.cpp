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

using namespace asclinic_pkg;

struct discreteParam
{
    float prev;
    float current;

    void updateNewValue()
    {
        prev = current;
    }
};

// Init global variables
AmrPose currentPose = AmrPose();
AmrPose refPose = AmrPose();
MotorAngularVelocity RotationAngleMsg;
discreteParam headingError;
discreteParam pathError;
discreteParam v;
discreteParam w;
discreteParam velocity_wheel_l;
discreteParam velocity_wheel_r;
const float samplingTime = 0.1;

ros::NodeHandle nodeHandle;
ros::Publisher publisher_4_internal_control;


void subscriberCallback4TrajRefSignal(const AmrPose& msg);
void subscriberCallback4OdoFeedback(const AmrPose& msg);
void updateRefPose(const AmrPose& msg);
float PIController4Heading(discreteParam error);
float PIController4PathFollowing(discreteParam error);


int main(int argc, char* argv[])
{
    ros::init(argc, argv, amr_node::TRAJ_TRACK_CONTROLLER);
    
    ros::Subscriber subscribe_odo_pose_feedback = nodeHandle.subscribe(amr_topic::POSE_DATA_FROM_ODOMETRY, 1, subscriberCallback4OdoFeedback);
	ros::Subscriber subscribe_traj_ref_signal = nodeHandle.subscribe(amr_topic::TRAJECTORY_REF_SIGNAL, 1, subscriberCallback4TrajRefSignal);

    ros::spin();
    

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
    
    headingError.updateNewValue();
    pathError.updateNewValue();

    headingError.current = atan2(refPose.pose_y-currentPose.pose_y,refPose.pose_x-currentPose.pose_x)-currentPose.pose_phi;
    pathError.current = sqrt(pow((refPose.pose_x - currentPose.pose_x),2)+pow((refPose.pose_y - currentPose.pose_y),2));

    v.updateNewValue();
    w.updateNewValue();
    
    v.current = PIController4PathFollowing(pathError);
    w.current = PIController4Heading(headingError);

    velocity_wheel_l.updateNewValue();
    velocity_wheel_r.updateNewValue();

    velocity_wheel_l.current = (v.current-AMR_BASE_HALF_WHEEL_BASE*w.current)/AMR_BASE_RADIUS_WHEEL_L;
    velocity_wheel_r.current = (v.current-AMR_BASE_HALF_WHEEL_BASE*w.current)/AMR_BASE_RADIUS_WHEEL_R;

    RotationAngleMsg.angular_velocity_motor_l = velocity_wheel_l.current;
    RotationAngleMsg.angular_velocity_motor_r = velocity_wheel_r.current;
    
    publisher_4_internal_control = nodeHandle.advertise<MotorAngularVelocity>(amr_topic::MOTOR_ANGULAR_VELOCITY_REF, 1, false);
    publisher_4_internal_control.publish(RotationAngleMsg);
}

/**
 * @brief update parameter currentPose of the robot obtained from odometry est
 * 
 * @param msg 
 */

void subscriberCallback4OdoFeedback(const AmrPose& msg)
{
    currentPose = msg;
}

void updateRefPose(const AmrPose& msg)
{
    refPose = msg;
}

/**
 * @brief PI controller for heading and path following
 * 
 * @param error 
 * @return float W from heading and V from pathFollowing
 */
float PIController4Heading(discreteParam error)
{
    float kp = 4;
    float ki = 4;
    return w.prev + error.current*(kp-ki*samplingTime) - kp*error.prev;
}

float PIController4PathFollowing(discreteParam error)
{
    float kp = 3;
    float ki = 0;
    return v.prev + error.current*(kp-ki*samplingTime) - kp*error.prev;
}