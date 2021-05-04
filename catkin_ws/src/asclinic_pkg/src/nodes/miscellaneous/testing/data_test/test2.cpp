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
    ros::init(argc, argv, "test_2");
    ros::NodeHandle nd;
    
    ros::Publisher publisher = nd.advertise<std_msgs::UInt32>("publisher", 10, false);  
    ros::Subscriber subscriber = nd.subscribe("publisher", 1, subscriberCallback);  

    ros::Rate loop_rate(1);
    // std_msgs::UInt32 msg;
    // int var = 0xffff;
    int counter = 266;

    
    while (ros::ok())
    {
        std_msgs::UInt32 msg;
        // new_var = (pre_var<<2) + 1<<1 + 0;
        new_var = ((pre_var<<2) + (1<<1) + 0)%16;
        
        msg.data = new_var;
        if (new_var == 0x0A)
            publisher.publish(msg);

        pre_var = new_var;
        ros::spinOnce();

        loop_rate.sleep();
    }
    
}