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
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt32.h"
#include "MotionPlanHeader.h"

using namespace asclinic_pkg;

ros::Publisher publisher_4_trajectory_tracking;
ros::Publisher publisher_state_changePath;
ros::Publisher publisher_4_testing_Matlab;
ros::Publisher publisher_4_emergency_stop;
ros::Publisher publisher_4_movement_type;
ros::Publisher publisher_4_currentPose;
ros::Publisher publisher_4_dynamic_state;
ros::Publisher publisher_4_load_unload;
ros::Publisher publisher_4_next_cycle;

std::string stateSentRequest;
amr_msgs::GeneralMsg state_change_request;
amr_msgs::RobotState LoadUnloadSignal;
int available_cycle = false;
std_msgs::Bool movement_type_msg;



pathStruct transitPath      = pathStruct();
pathStruct plannedPath      = pathStruct();
pathStruct testPath         = pathStruct();
pathStruct Back2StartPath   = pathStruct();

void subscriberCallback4TransitPath(const geometry_msgs::Polygon& msg);
void subscriberCallback4PlannedPath(const geometry_msgs::Polygon& msg);
void subscriberCallback4TestPath(const geometry_msgs::Polygon& msg);
void subscriberCallback4Back2StartPath(const geometry_msgs::Polygon& msg);
// void testingMatlab(const std_msgs::Float64& msg);
void getStateIndicator(const std_msgs::String& msg);
AmrPose toAmrPose(geometry_msgs::Point32 data);
void subscriberCallback4SwitchState(const amr_msgs::GeneralMsg& msg);
void subscriberCallback4notifyCycle(const std_msgs::UInt32& msg);


std_msgs::Bool in_moving_msg;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, amr_node::MOTION_PLANNING);
    ros::NodeHandle nodeHandle;

	ros::Subscriber subscribe_transit_path  = nodeHandle.subscribe(amr_topic::TRANSIT_PATH_MATLAB, 1, subscriberCallback4TransitPath);
    ros::Subscriber subscribe_planned_path  = nodeHandle.subscribe(amr_topic::PLANNED_PATH_MATLAB, 1, subscriberCallback4PlannedPath);
    ros::Subscriber subscribe_test_path     = nodeHandle.subscribe("test_path", 1, subscriberCallback4TestPath);
    ros::Subscriber subscribe_back2start_path = nodeHandle.subscribe("back2start_path", 1, subscriberCallback4Back2StartPath);
    
    ros::Subscriber state_indicator         = nodeHandle.subscribe(amr_topic::STATE, 1, getStateIndicator);
    ros::Subscriber subscribe_4_switch_state= nodeHandle.subscribe(amr_topic::STATE_COMPLETED_SIGNAL, 1, subscriberCallback4SwitchState);
    ros::Subscriber subscribe_4_notify_cycle= nodeHandle.subscribe("notify_available_cycle",1, subscriberCallback4notifyCycle);

    publisher_4_trajectory_tracking         = nodeHandle.advertise<AmrPose>(amr_topic::TRAJECTORY_REF_SIGNAL, 1, false);
    publisher_4_emergency_stop              = nodeHandle.advertise<MotorAngularVelocity>(amr_topic::MOTOR_ANGULAR_VELOCITY_REF, 1, false);
    publisher_4_movement_type               = nodeHandle.advertise<std_msgs::Bool>(amr_topic::MOVEMENT_TYPE, 1, false);
    publisher_4_currentPose                 = nodeHandle.advertise<AmrPose>(amr_topic::POSE_DATA_FROM_ODOMETRY, 1, true);
    publisher_4_dynamic_state               = nodeHandle.advertise<std_msgs::Bool>("dynamic_state", 1, true);
    publisher_4_load_unload                 = nodeHandle.advertise<amr_msgs::RobotState>(amr_topic::LOAD_UNLOAD_STATE_SIGNAL, 1, true);
    publisher_4_next_cycle                  = nodeHandle.advertise<std_msgs::UInt32>("get_next_cycle", 1, true);
    ros::Publisher publisher_4_in_movement = nodeHandle.advertise<std_msgs::Bool>("in_movement", 1, true);
    // need to convert point 32 to AmrPose
    // freq = 2Hz, T = 0.5s
    ros::Rate rate(1/AMR_TIMER_4_POSE_REF_SIGNAL);
    
    // index of the path list
    int index = 0;
    while (ros::ok())
    {
        // IDLE STATE
        
        if (!Robot_State.compare(amr_state::IDLE))
        {
            ROS_INFO_STREAM("[State: " << Robot_State << "]" << std::endl);

            // a transition state for other states
            if (available_cycle)
            {
                std_msgs::UInt32 msg;
	            msg.data = 1;

                // publish for requesting process the list of cycle
                ROS_INFO_STREAM("[IDLE] REQUESTING PROCESSING CYCLE"<< std::endl);
                publisher_4_next_cycle.publish(msg);
            }
            else
            {
                ROS_INFO_STREAM("[IDLE] UNAVAILABLE CYCLE"<< std::endl);
            }

            if (transitPath.noPoints)
            {
                Robot_State = amr_state::IN_TRANSIT;

                in_moving_msg.data = false;
                available_cycle--;
                publisher_4_in_movement.publish(in_moving_msg);
            }
        }

        // REQUEST STATE (NO STATE ATM)
        else if (!Robot_State.compare(amr_state::REQUEST_STATE))
        {
            ROS_INFO_STREAM("[State: " << Robot_State << "]" << std::endl);
            // user needs to publish "idle" so that it can be ready to do task

            in_moving_msg.data = false;
            publisher_4_in_movement.publish(in_moving_msg);
        }

        // EMERGENCY STOP
        else if (!Robot_State.compare(amr_state::EMERGENCY_STOP))
        {
            ROS_INFO_STREAM("[State: " << Robot_State << "]" << std::endl);
            // send command for motor control to stop
            // record all the current data of the robot, for resuming the action

            RotationAngleMsg.angular_velocity_motor_l = 0;
            RotationAngleMsg.angular_velocity_motor_r = 0;

            ROS_INFO_STREAM("[MOTION PLANNING] request angular vel (L/ R)"<< RotationAngleMsg.angular_velocity_motor_l << "; "<<RotationAngleMsg.angular_velocity_motor_r);
            publisher_4_emergency_stop.publish(RotationAngleMsg);
            ROS_INFO_STREAM("[MOTION PLAN: EMERGENCY STOP] --- Require user command ---");

            in_moving_msg.data = false;
            publisher_4_in_movement.publish(in_moving_msg);
        }

        // MOVING TO THE PICK UP POINT
        else if (!Robot_State.compare(amr_state::IN_TRANSIT))
        {

            if (index <= transitPath.noPoints-1)
            {
                ROS_INFO_STREAM("[State: "  << Robot_State << "]" << std::endl);
                ROS_INFO_STREAM("[IN TRANSIT] Index: "   << index << std::endl);
                // follow the transit path to move to the transit point
                // send the reference point for very 0.5s

                // Check for pure rotation movement
                // by comparing 2 heading values
                // rotation 1
                movement_type_msg.data = 0;
                if (index)
                {
                    // check for rotation
                    if (((transitPath.path.points[index].x == transitPath.path.points[index-1].x)) && ((transitPath.path.points[index].y == transitPath.path.points[index-1].y)) && ((transitPath.path.points[index].z == transitPath.path.points[index-1].z)))
                    {
                        continue;
                    }
                    else
                    {
                        movement_type_msg.data = (transitPath.path.points[index].z != transitPath.path.points[index-1].z);
                    }
                }
                else if (index ==0)
                {
                    movement_type_msg.data = 1;
                }

                publisher_4_movement_type.publish(movement_type_msg);


                // Increment index
                publisher_4_trajectory_tracking.publish(toAmrPose(transitPath.path.points[index]));
                index++;


                in_moving_msg.data = true;
                publisher_4_in_movement.publish(in_moving_msg);
            }
            else 
            {
                Robot_State = amr_state::ARRIVE_AT_TRANSIT;
                index = 0;
                std_msgs::Bool msg;
                msg.data = false;
                publisher_4_in_movement.publish(msg);
            }
        }

        // ARRIVED AT THE PICK UP POINT
        else if (!Robot_State.compare(amr_state::ARRIVE_AT_TRANSIT))
        {
            ROS_INFO_STREAM("[State: " << Robot_State << "]" << std::endl);

            // Set 0 velocity to stop the robot
            RotationAngleMsg.angular_velocity_motor_l = 0;
            RotationAngleMsg.angular_velocity_motor_r = 0;
            publisher_4_emergency_stop.publish(RotationAngleMsg);

            // Send signal for requesting state Load
            LoadUnloadSignal.data = FSM_State::LOAD;
            ROS_INFO_STREAM("[LOADING STUFF] signal: "<< LoadUnloadSignal.data);
            
            publisher_4_load_unload.publish(LoadUnloadSignal);
            

            // activate the actuator for pick up things
            // send a command to indicate if the things have been succesffully picked
            // send a command to traj track for using planned path
            // transit to in delivery state
            // std_msgs::Bool msg;
            // msg.data = 1;
            // publisher_4_in_movement.publish(msg);

            in_moving_msg.data = false;
            publisher_4_in_movement.publish(in_moving_msg);
        }

        // MOVING TO THE DELIVERY POINT
        else if (!Robot_State.compare(amr_state::IN_DELIVERY))
        {
            // Need to follow the path, the traj track will send data from
            // the planned path list, send the reference point for very 0.5s
            // check if the robot has successfully reached to the final point

            // Change to the last number in the square path index -2

            std_msgs::Bool msg;
            msg.data = 1;
            publisher_4_in_movement.publish(msg);

            if (index <= plannedPath.noPoints-1)
            {
                ROS_INFO_STREAM("[State: " << Robot_State << "]" << std::endl);
                ROS_INFO_STREAM("[IN DELIVERY] Index: "<< index << std::endl);
                // follow the transit path to move to the transit point
                // send the reference point for very 0.5s


                // Check for pure rotation movement
                // by comparing 2 heading values
                movement_type_msg.data = 0;
                if (index)
                {
                    // check for rotation
                    if (((plannedPath.path.points[index].x == plannedPath.path.points[index-1].x)) && ((plannedPath.path.points[index].y == plannedPath.path.points[index-1].y)) && ((plannedPath.path.points[index].z == plannedPath.path.points[index-1].z)))
                    {
                        continue;
                    }
                    else
                    {
                        movement_type_msg.data = (plannedPath.path.points[index].z != plannedPath.path.points[index-1].z);
                    }
                }
                else if (index ==0)
                {
                    movement_type_msg.data = 1;
                }

                publisher_4_movement_type.publish(movement_type_msg);

                // reset index
                publisher_4_trajectory_tracking.publish(toAmrPose(plannedPath.path.points[index]));
                index++;

                in_moving_msg.data = true;
                publisher_4_in_movement.publish(in_moving_msg);
            }
            else 
            {
                // then change to arrived at final state
                Robot_State = amr_state::ARRIVE_AT_FINAL;
                index = 0;

            }

        }

        // ARRIVED AT THE DELIVERY POINT
        else if (!Robot_State.compare(amr_state::ARRIVE_AT_FINAL))
        {
            ROS_INFO_STREAM("[State: " << Robot_State << "]" << std::endl);

            // Set 0 velocity to stop the robot
            RotationAngleMsg.angular_velocity_motor_l = 0;
            RotationAngleMsg.angular_velocity_motor_r = 0;
            publisher_4_emergency_stop.publish(RotationAngleMsg);
            
            // Send signal for requesting state unLoad
            LoadUnloadSignal.data = FSM_State::UNLOAD;
            publisher_4_load_unload.publish(LoadUnloadSignal);
            
            // activate the actuator for drop off things
            // send command to make sure the things have been left
            // if there is more things to collect -> transit state, update transit path
            // else move to initial position -> "new" state -> idle state      
            in_moving_msg.data = false;
            publisher_4_in_movement.publish(in_moving_msg);      
        }

        // GET BACK TO START
        else if (!Robot_State.compare(amr_state::GET_BACK_TO_START))
        {
            ROS_INFO_STREAM("[State: " << Robot_State << "]" << std::endl);
            // std_msgs::Bool msg;
            // msg.data = 1;
            // publisher_4_in_movement.publish(msg);
            
            // activate the actuator for drop off things
            // send command to make sure the things have been left
            // if there is more things to collect -> transit state, update transit path
            // else move to initial position -> "new" state -> idle state      
            if (index <= Back2StartPath.noPoints-1)
            {
                ROS_INFO_STREAM("[State: " << Robot_State << "]" << std::endl);
                ROS_INFO_STREAM("[IN DELIVERY] Index: "<< index << std::endl);
                // follow the transit path to move to the transit point
                // send the reference point for very 0.5s


                // Check for pure rotation movement
                // by comparing 2 heading values
                movement_type_msg.data = 0;
                if (index)
                {
                    movement_type_msg.data = (Back2StartPath.path.points[index].z != Back2StartPath.path.points[index-1].z);
                }

                publisher_4_movement_type.publish(movement_type_msg);

                // reset index
                publisher_4_trajectory_tracking.publish(toAmrPose(Back2StartPath.path.points[index]));
                index++;

                in_moving_msg.data = true;
                publisher_4_in_movement.publish(in_moving_msg);
            }
            else // IDLE STATE
            {
                // then change to arrived at final state
                Robot_State = amr_state::IDLE;
                index = 0;

                in_moving_msg.data = false;
                publisher_4_in_movement.publish(in_moving_msg);
            }      
        }

        // INVALID STATE INDICATOR
        else
        {
            ROS_INFO_STREAM("-- INVALID STATE INDICATOR --" << std::endl);
        }
        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}

// ---------- ADDITIONAL FUNCTIONS FOR GETTING PATHS ---------------

void subscriberCallback4TransitPath(const geometry_msgs::Polygon& msg)
{
    ROS_INFO_STREAM("[GETTING TRANSIT PATH] transit Path: " << std::endl);
    transitPath.importPolygonMSG(msg);
    transitPath.isAvailable("transit Path");  
}

void subscriberCallback4PlannedPath(const geometry_msgs::Polygon& msg)
{
    ROS_INFO_STREAM("[GETTING PLANNED PATH] planned Path: " << std::endl);
    plannedPath.importPolygonMSG(msg);
    plannedPath.isAvailable("planned Path");
}

void subscriberCallback4TestPath(const geometry_msgs::Polygon& msg)
{
    ROS_INFO_STREAM("[GETTING TEST PATH] test Path: " << std::endl);
    testPath.importPolygonMSG(msg);
    ROS_INFO_STREAM("Last point: " << testPath.path.points[23].x);
    ROS_INFO_STREAM("Last point: " << testPath.path.points[23].y);
    ROS_INFO_STREAM("POINTS: " << testPath.noPoints << std::endl);
    testPath.isAvailable("test Path");
}

void subscriberCallback4Back2StartPath(const geometry_msgs::Polygon& msg)
{
    ROS_INFO_STREAM("[GETTING back2start PATH] back2start Path: " << std::endl);
    Back2StartPath.importPolygonMSG(msg);
    Back2StartPath.isAvailable("back 2 start Path");
}
// --------------------------------------------------------------------


AmrPose toAmrPose(geometry_msgs::Point32 data)
{
    AmrPose msgPose;
    msgPose.pose_x = data.x;
    msgPose.pose_y = data.y;
    msgPose.pose_phi = data.z;
    return msgPose;
}


void getStateIndicator(const std_msgs::String& msg)
{
    stateSentRequest = msg.data;

    if (!stateSentRequest.compare("IDLE") || !stateSentRequest.compare("idle"))
    {
        Robot_State = amr_state::IDLE;
        ROS_INFO_STREAM(" -- Switch state -- to "<< "IDLE" << std::endl);
    }
    else if (!stateSentRequest.compare("STOP") || !stateSentRequest.compare("stop"))
    {
        Robot_State = amr_state::EMERGENCY_STOP;
        ROS_INFO_STREAM(" -- Switch state -- to "<< "EMG STOP" << std::endl);
        RotationAngleMsg.angular_velocity_motor_l = 0;
        RotationAngleMsg.angular_velocity_motor_r = 0;

        publisher_4_emergency_stop.publish(RotationAngleMsg);
    }
    else if (!stateSentRequest.compare("start_transit") || !stateSentRequest.compare("START_TRANSIT"))
    {
        Robot_State = amr_state::IN_TRANSIT;
        ROS_INFO_STREAM(" -- Switch state -- to "<< "IN TRANSIT" << std::endl);
    }
    else if (!stateSentRequest.compare("arrived at transit") || !stateSentRequest.compare("ARRIVED AT TRANSIT"))
    {
        Robot_State = amr_state::ARRIVE_AT_TRANSIT;
        ROS_INFO_STREAM(" -- Switch state -- to "<< "ARRIVE AT TRANSIT" << std::endl);
    }
    else if (!stateSentRequest.compare("start_delivery") || !stateSentRequest.compare("START_DELIVERY"))
    {
        Robot_State = amr_state::IN_DELIVERY;
        ROS_INFO_STREAM(" -- Switch state -- to "<< "IN DELIVERY" << std::endl);
    }
    else if (!stateSentRequest.compare("arrived at final") || !stateSentRequest.compare("ARRIVED AT FINAL"))
    {
        Robot_State = amr_state::ARRIVE_AT_FINAL;
        ROS_INFO_STREAM(" -- Switch state -- to "<< "ARRIVE AT FINAL" << std::endl);
    }
    else if (!stateSentRequest.compare("get back to start") || !stateSentRequest.compare("GET BACK TO START"))
    {
        Robot_State = amr_state::GET_BACK_TO_START;
        ROS_INFO_STREAM(" -- Switch state -- to "<< "GET_BACK_TO_START" << std::endl);
    }
    else
    {
        ROS_INFO_STREAM("-- INVALID STATE INDICATOR --" << std::endl);
    }
    
}

void subscriberCallback4SwitchState(const amr_msgs::GeneralMsg& msg)
{
    if (!Robot_State.compare(amr_state::ARRIVE_AT_TRANSIT))
    {
        Robot_State = amr_state::IN_DELIVERY;
        ROS_INFO_STREAM(" -- Switch state -- to "<< "IN DELIVERY" << std::endl);
    }
    else if (!Robot_State.compare(amr_state::ARRIVE_AT_FINAL))
    {
        Robot_State = amr_state::GET_BACK_TO_START;
        ROS_INFO_STREAM(" -- Switch state -- to "<< "GET_BACK_TO_START" << std::endl);
    }
}

void subscriberCallback4notifyCycle(const std_msgs::UInt32& msg)
{
    if (msg.data)
    {
        ROS_INFO_STREAM("[MOTION PLANNING] --- There is(are) "<< msg.data <<" cycle available ---" << std::endl);
        available_cycle = msg.data;
    }
    else
    {
        ROS_INFO_STREAM("[MOTION PLANNING] --- NO CYCLE YET ---"<< std::endl);
    }
}