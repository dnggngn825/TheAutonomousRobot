#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"

#include <bitset>
#include "amr/amr.h"

using namespace asclinic_pkg;

void subscriberCallback(const std_msgs::UInt32& msg)
{
    ROS_INFO_STREAM("Number: "<<msg.data);
}

int main(int argc,  char* argv[])
{
    ros::init(argc, argv, "test_1");
    ros::NodeHandle nd;
    
    ros::Publisher publisher = nd.advertise<std_msgs::UInt32>("publisher", 10, false);  
    ros::Subscriber subscriber = nd.subscribe("publisher", 1, subscriberCallback);  

    ros::Rate loop_rate(1);

    int encoder_counter = 26;
    float sampling_time_in_sec = AMR_TIMER_4_SYSTEM_CONTROL_IN_SEC;
    float angular_velocity_rpm = (encoder_counter*1.0/POLOLU_SMC_G2_ENCODER_RESOLUTION)/sampling_time_in_sec*60.0;
    float angular_velocity_pwm_pre = 100.0f*angular_velocity_rpm/POLOLU_SMC_G2_MOTOR_MAX_ANGULAR_VELOCITY_RPM_W_GEARBOX;
    int angular_velocity_pwm = round(angular_velocity_pwm_pre);

    ROS_INFO_STREAM("sampling_time_in_sec "<< sampling_time_in_sec);
    ROS_INFO_STREAM("angular_velocity_rpm "<< angular_velocity_rpm);
    // ROS_INFO_STREAM("POLOLU_SMC_G2_MOTOR_MAX_ANGULAR_VELOCITY_RPM_W_GEARBOX "<< POLOLU_SMC_G2_MOTOR_MAX_ANGULAR_VELOCITY_RPM_W_GEARBOX);
    // ROS_INFO_STREAM("angular_velocity_pwm_pre "<< angular_velocity_pwm_pre);
    // ROS_INFO_STREAM("angular_velocity_pwm "<< angular_velocity_pwm);


    // ROS_INFO_STREAM("AMR_TIMER_4_MOTOR_CONTROL_IN_SEC "<< AMR_TIMER_4_MOTOR_CONTROL_IN_SEC);
    // ROS_INFO_STREAM("AMR_TIMER_4_MOTOR_CONTROL_IN_MIN "<< AMR_TIMER_4_MOTOR_CONTROL_IN_MIN);
    // ROS_INFO_STREAM("1/AMR_TIMER_4_MOTOR_CONTROL_IN_MIN "<< 1.0f/AMR_TIMER_4_MOTOR_CONTROL_IN_MIN);
    // ROS_INFO_STREAM("AMR_TIMER_4_SYSTEM_CONTROL_IN_SEC "<< AMR_TIMER_4_SYSTEM_CONTROL_IN_SEC);
    // ROS_INFO_STREAM("AMR_TIMER_4_SYSTEM_CONTROL_IN_MIN "<< AMR_TIMER_4_SYSTEM_CONTROL_IN_MIN);
    // ROS_INFO_STREAM("1/AMR_TIMER_4_SYSTEM_CONTROL_IN_MIN "<< 1/AMR_TIMER_4_SYSTEM_CONTROL_IN_MIN);
    // ROS_INFO_STREAM("POLOLU_SMC_G2_ENCODER_RESOLUTION "<< POLOLU_SMC_G2_ENCODER_RESOLUTION);
    // ROS_INFO_STREAM("1/POLOLU_SMC_G2_ENCODER_RESOLUTION "<< 1.0f/POLOLU_SMC_G2_ENCODER_RESOLUTION);
    // ROS_INFO_STREAM("POLOLU_SMC_G2_ENCODER_MAX_DEBOUNCE_DURATION_SESC_W_GEARBOX "<< POLOLU_SMC_G2_ENCODER_MAX_DEBOUNCE_DURATION_SESC_W_GEARBOX);

}