//
// Created by newnottakenname on 15.01.16.
//

#include <ros/rate.h>
#include <ros/node_handle.h>
#include "DynamixelMotor.h"
#include <dynamixel_msgs/JointState.h>
#include <std_msgs/Float64.h>

int rotations;

double lastPosition;
double wantedPosition;
double startPosition;
double radius;
double speedModifier = 5;
double minus;

const double oneRotation = 2 * M_PI;


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
    lastPosition = 0;


}

void DynamixelMotor::setMotorName(std::string name, double rad)
{
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
    int i = 0;
    while(i < 3){
        i++;
        ros::spinOnce();
        ros::Rate sleep(1);
        sleep.sleep();
        ROS_INFO(std::to_string(i).c_str());
    }


    startPosition = lastPosition;
}

void DynamixelMotor::runIntoLimit()
{
    setSpeed(-0.7);



    initCompleted();
}

void DynamixelMotor::updateSpeed()
{
    if(startPosition == 0.0){
        return;
    }
    double fullLastPosition = oneRotation * rotations + lastPosition;

    double changeInPosition = (wantedPosition/radius) - fullLastPosition;

    if(fabs(changeInPosition) < 0.01 )
    {
        setSpeed(0);
    }
    else
    {
        setSpeed(changeInPosition * speedModifier);
//        ROS_INFO("CIP");
//        ROS_INFO(std::to_string(changeInPosition).c_str());
//        ROS_INFO("lastPos");
        ROS_INFO(std::to_string(lastPosition).c_str());
//        ROS_INFO("rot");
        ROS_INFO(std::to_string(rotations).c_str());
    }



}

void DynamixelMotor::loopOnce()
{
    ros::spinOnce();
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

}

void DynamixelMotor::setMinus(double d)
{
    minus = d;

}

void DynamixelMotor::gotoPosition(double position)
{
    wantedPosition = position;
}

void DynamixelMotor::positionCallback(const dynamixel_msgs::JointState::ConstPtr &mes) {
    double cur_pos = mes->current_pos + minus;

    if(lastPosition != 0){
        if(fabs(cur_pos - lastPosition) > 2)
        {
            setSpeed(0);
            if((lastPosition - cur_pos) > 0){
                rotations++;
            }
            else
            {
                rotations--;
            }
        }
    }
    lastPosition = cur_pos;
    updateSpeed();
}
