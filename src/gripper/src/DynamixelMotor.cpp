//
// Created by newnottakenname on 15.01.16.
//

#include <ros/rate.h>
#include <ros/node_handle.h>
#include "DynamixelMotor.h"
#include <dynamixel_msgs/JointState.h>
#include <std_msgs/Float64.h>



void DynamixelMotor::loop()
{
//    MX_Pub.publish(mes);
//
//    ros::NodeHandle nh_;
//
//    //double d = (2*M_PI + 0.4*M_PI);
//    //int i = (int) d;
//
//    //std::this_thread::sleep_for(std::chrono::microseconds(1));
//
//    ros::Rate waitTime(0.5);
//    waitTime.sleep();
//
//    //loop_rate.sleep();
//
//    mes.data = 0;
//
//    MX_Pub.publish(mes);
//
//    loop_rate.sleep();
//    loop_rate.sleep();
//    loop_rate.sleep();
//
//    updateSpeed();

}

void DynamixelMotor::init()
{
    wantedPosition = 0;
    currentPosition = 0;
    rotations = 0;

}

void DynamixelMotor::setMotorName(std::string name, double rad, int ids)
{
    id = ids;
    radius = rad;
    MX_Sub = nh_.subscribe(name + "state", 50, &DynamixelMotor::positionCallback, this);
    MX_Pub = nh_.advertise<std_msgs::Float64>(name + "command", 1000);

    int i = 0;
    while(ros::ok()){

        std_msgs::Float64 mes2;

        mes2.data = 0;

        MX_Pub.publish(mes2);

        if(i == 20){
            break;
        }

        i++;

        ros::spinOnce();

        ros::Rate sleep(100);
        sleep.sleep();
    }

}

void DynamixelMotor::initCompleted()
{
    while(currentPosition == 0.0){
        ros::spinOnce();
    }

    startPosition = currentPosition;
    wantedPosition = startPosition;
    rotations = 0;
    completed = true;
    initiased = true;

}

void DynamixelMotor::runIntoLimit()
{
    setSpeed(-0.7);
    ros::Rate sleep(0.7);
    sleep.sleep();

//    ros::Rate waitRate(0.1);
//    waitRate.sleep();
}

void DynamixelMotor::limitCheck() {
    double diff = fabs(lastPosition - currentPosition);
    if(diff < 0.01 && currentPosition != 0.0)
    {
        initCompleted();
        completed = true;
    }
}

void DynamixelMotor::setManualControl(bool on)
{
    manualControl = on;
}

bool DynamixelMotor::hasCompleted()
{
    return completed;
}


void DynamixelMotor::loopOnce()
{
}

void DynamixelMotor::setMultiplier(double d)
{
    speedModifier = d;
}


void DynamixelMotor::setSpeed(double d)
{
    std_msgs::Float64 mes;
    mes.data = d;
    MX_Pub.publish(mes);
    //manualControl = false;

}

void DynamixelMotor::setOneRotation(double rad)
{
    oneRotation = rad;
}

void DynamixelMotor::gotoPosition(double position)
{
    completed = false;
    wantedPosition = (position/radius) + startPosition;
}

void DynamixelMotor::positionCallback(const dynamixel_msgs::JointState::ConstPtr &mes) {
    double cur_pos = static_cast<double>(mes->current_pos);

    if(cur_pos < 0.001)
    {
        int i = 0;
    }

    if(fabs(currentPosition) > 1E-31){
        if(fabs(cur_pos - currentPosition) > 2)
        {
            if((currentPosition - cur_pos) > 0){
                rotations++;
            }
            else
            {
                rotations--;
            }
        }
    }
    lastPosition = currentPosition;
    currentPosition = cur_pos;
    if(initiased) {
        updateSpeed();
    }
    else
    {
        limitCheck();
    }
}

void DynamixelMotor::updateSpeed()
{
    if(startPosition == 0.0 | manualControl){
        return;
    }
    double fullLastPosition = oneRotation * rotations + currentPosition;

    double changeInPosition = wantedPosition - fullLastPosition;

    if(fabs(changeInPosition) < 0.01 )
    {
        setSpeed(0);
        completed = true;
    }
    else
    {
        completed = false;
        setSpeed(changeInPosition * speedModifier);
//        ROS_INFO("CIP");
//        ROS_INFO(std::to_string(changeInPosition).c_str());
//        ROS_INFO("lastPos");
        std::string info = "Postition: ";
        info.append(std::to_string(fullLastPosition));
        info.append("\t Wanted: ");
        info.append(std::to_string(wantedPosition));
        info.append("\t Rotations: ");
        info.append(std::to_string(rotations));
        ROS_INFO(info.c_str());
    }



}


