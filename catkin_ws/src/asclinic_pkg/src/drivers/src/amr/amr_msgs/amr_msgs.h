/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


#include "std_msgs/UInt32.h"
#include "std_msgs/UInt16.h"
#include "asclinic_pkg/AmrPose.h"
#include "asclinic_pkg/MotorRotationAngle.h"
#include "asclinic_pkg/MotorAngularVelocityPWM.h"
#include "asclinic_pkg/MotorAngularVelocity.h"

using namespace asclinic_pkg;

#ifndef AMR_MSGS_H
#define AMR_MSGS_H


namespace amr_msgs
{
    typedef std_msgs::UInt32 TimerID;
    typedef std_msgs::UInt16 EncCounter;
}

#endif 
