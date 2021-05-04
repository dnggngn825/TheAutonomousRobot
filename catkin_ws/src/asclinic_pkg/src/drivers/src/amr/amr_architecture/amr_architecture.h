/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


#ifndef AMR_ARCHITECTURE_H
#define AMR_ARCHITECTURE_H


namespace amr_topic 
{
    const char* TIMER_4_MOTOR_CONTROL               = "timer_4_motor_control";
    const char* TIMER_4_SYSTEM_CONTROL              = "timer_4_system_control";
    
    const char* POSE_REF                            = "pose_reference";
    const char* POSE_EST                            = "pose_estimated";

    const char* MOTOR_ANGULAR_VELOCITY_REF          = "motor_angular_velocity_reference";
    const char* MOTOR_ANGULAR_VELOCITY_EST          = "motor_angular_velocity_estimated";
    
    const char* MOTOR_PWM_DUTY_CYCLE                = "motor_pwm_duty_cycle";

    const char* ENCODER_DATA_4_MOTOR_CONTROL        = "encoder_data_motor_control";
    const char* ENCODER_DATA_4_MOTOR_CONTROL_L      = "encoder_data_motor_control_l";
    const char* ENCODER_DATA_4_MOTOR_CONTROL_R      = "encoder_data_motor_control_r";
    const char* ENCODER_DATA_4_ODOMETRY_CONTROL     = "encoder_data_odometry_control";
    const char* ENCODER_DATA_4_ODOMETRY_CONTROL_L   = "encoder_data_odometry_control_l";
    const char* ENCODER_DATA_4_ODOMETRY_CONTROL_R   = "encoder_data_odometry_control_r";

    const char* POSE_DATA_FROM_ODOMETRY             = "estimated_pose_data_from_odometry";
    const char* POSE_DATA_FROM_IP                   = "estimated_pose_from_ip";

    const char* TRANSIT_PATH_MATLAB                 = "transit_path_Matlab";
    const char* PLANNED_PATH_MATLAB                 = "planned_path_Matlab";
    const char* TRAJECTORY_REF_SIGNAL               = "trajectory_ref_signal";
}

namespace amr_node 
{
    const char* TIMER_MONITOR                       = "timer_monitor";
    const char* POSE_CONTROLLER                     = "pose_controller";
    const char* MOTOR_CONTROLLER                    = "motor_controller";
    const char* MOTOR_DYNAMICS                      = "motor_dynamics";
    const char* ENCODER_MONITOR                     = "encoder_monitor";
    const char* ENCODER_MONITOR_4_MOTOR_CONTROL     = "enc_monitor_4_motor_ctl";
    const char* ENCODER_MONITOR_4_ODOM_CONTROL      = "enc_monitor_4_odom_ctl";
    const char* ENCODER_MONITOR_L                   = "encoder_monitor_l";
    const char* ENCODER_MONITOR_R                   = "encoder_monitor_r";
    const char* ODOMETRY_ESTIMATOR                  = "odometry_estimator"; 
    const char* MOTION_PLANNING                     = "motion_planning";
    const char* TRAJ_TRACK_CONTROLLER               = "traj_track_controller";
}

#endif

