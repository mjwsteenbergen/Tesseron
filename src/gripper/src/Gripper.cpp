//
// Created by newnottakenname on 15.12.15.
//

#include "Gripper.h"
#include "gripper/LayDown.h"
#include "gripper/PickUp.h"
#include "gripper/MoveGripper.h"
#include "gripper/StatusMessage.h"

#include "dynamixel_controllers/StartController.h"
#include "dynamixel_controllers/SetSpeed.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "DynamixelMotor.h"
#include <iostream>
#include <string>
#include <math.h>
#include <thread>

bool initialised = false;

void Gripper::sendErrorMessage(const std::string message_body, const std::string sender) {
//    ROS_WARN(message_body.c_str(), "Gripper");
//    gripper::StatusMessage message;
//    message.completed = (unsigned char) false;
//    message.error = message_body;
//    message.sender = sender;
//    initPublisher.publish(message);
}


void Gripper::init()
{
    LaydownClient = nh_.advertiseService("/laydown", &Gripper::handleLayDown, this);
    PickUpClient  = nh_.advertiseService("/pickup" , &Gripper::handlePickUp, this );
    MoveClient    = nh_.advertiseService("/Gripper" ,&Gripper::handleMove, this );

    speedTopic = nh_.subscribe("/grippercontrol", 1, &Gripper::TwistCallback, this);

    //StatusPublisher
    initPublisher = nh_.advertise<std_msgs::Bool>("/Tesseron/init", 10);

    if(initialise())
    {
        ros::Rate rate(1);
        ros::spinOnce();
        rate.sleep();

        std_msgs::Bool msg;
        msg.data = true;
        initPublisher.publish(msg);
    }



    //resetArms();
}



bool Gripper::initialise() {
    RX.setMotorName("/tilt_controller/", 15E-3, 1);
    RX.initCompleted();
    handlePickup(true);

    ros::Rate sleep(5);
    for (int i = 0; i < 10; ++i) {
        sleep.sleep();
        loopOnce();
        while(!RX.hasCompleted())
        {
            loopOnce();
        }
    }

    MX.setMotorName("/pan_controller/", 3E-02, 2);
    MX.runIntoLimit();

    while(!MX.hasCompleted())
    {
        loopOnce();
    }
    for (int i = 0; i < 10; ++i) {
        sleep.sleep();
        loopOnce();
        while(!RX.hasCompleted() | !MX.hasCompleted())
        {
            loopOnce();
        }
    }

    spindle.initialise();
    spindle.intialiseExternal();
    spindle.isInitialised();
    while(!spindle.isInitialised())
    {
        loopOnce();
    }

    initialised = true;

    return true;
}

void Gripper::BlockingRX(double distance)
{
    RX.gotoPosition(distance);
    while(!RX.hasCompleted())
    {
        loopOnce();
    }
}

void Gripper::BlockingMX(double distance)
{
    MX.gotoPosition(distance);
    while(!MX.hasCompleted())
    {
        loopOnce();
    }
}

void Gripper::BlockingSpindle(double distance)
{
    spindle.gotoPosition(distance);
    while(!spindle.hasCompleted())
    {
        loopOnce();
    }
}

bool Gripper::handleLayDown(gripper::LayDown::Request &req, gripper::LayDown::Response &res)
{
    //TODO

    handleLayDown(req.fully);

    res.succeeded = (unsigned char) false;
    return true;
}



void Gripper::handleLayDown(bool fully) {
    if(fully)
    {
        BlockingRX(0.00);
    }
    else{
        BlockingRX(0.003);
    }
}


bool Gripper::handlePickUp(gripper::PickUp::Request &req, gripper::PickUp::Response &res)
{
    handlePickup(req.fully);

    res.succeeded = (unsigned char) true;
    return true;
}


void Gripper::handlePickup(bool fully) {
    if(fully)
    {
        BlockingRX(0.124);
    }
    else
    {
        BlockingRX(0.119);
    }
}

bool Gripper::handleMove(gripper::MoveGripper::Request &req, gripper::MoveGripper::Response &res)
{
    res.succeeded = (unsigned char) true;

    handleMove(req.x, req.y);


    return true;
}

bool Gripper::handleMove(double spindlePos, double MXPos)
{
    spindle.gotoPosition(spindlePos);
    MX.gotoPosition(MXPos);

    while(!spindle.hasCompleted() || !MX.hasCompleted())
    {
        loopOnce();
    }

}

void Gripper::loopOnce(){
    MX.loopOnce();
    RX.loopOnce();
    spindle.loopOnce();

    ros::spinOnce();
}





void Gripper::resetArms() {
    //TODO

//    if(resetGearBar() && resetSpindle()) { //Something went wrong
//        gripper::StatusMessage message;
//        message.completed = (unsigned char) true;
//        message.error = "";
//        message.sender = "Gripper";
//        initPublisher.publish(message);
//    }
//    sendErrorMessage("Not Implemented", "Gripper");
}

void Gripper::setSpeed(double speedMX, double speedSpindle, double speedRX)
{
    spindle.setSpeed(speedSpindle);
    RX.setSpeed(speedMX);
    MX.setSpeed(speedRX);
}

void Gripper::moveToShutdown(){
    spindle.gotoPosition(-0.03);
    while(!spindle.hasCompleted()){
        loopOnce();
    }
    MX.gotoPosition(0.05);
    while(!MX.hasCompleted())
    {
        loopOnce();
    }
    RX.gotoPosition(0.001);
    while(!RX.hasCompleted()){
        loopOnce();
    }
}

void Gripper::loop()
{
    while(ros::ok()){
        loopOnce();
    }

    //STOP();
}

int main(int argc, char **argv)
{
    ROS_INFO("BOOTING...");
    ros::init(argc, argv, "Gripper");
    Gripper grip;
//    grip.MX.gotoPosition(0.2);
//    grip.handleLayDown(true);
//    grip.handleMove(-0.16,0.13);
//    grip.BlockingRX(0.119);


//    ros::Rate sleep(0.1);
//    sleep.sleep();

//    grip.handlePickup(true);
//    grip.moveToShutdown();
    grip.loop();
//    gripper::MoveGripperRequest req;

}

void Gripper::TwistCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    if(initialised)
    {
        RX.setManualControl(true);
        MX.setManualControl(true);
        spindle.setManualControl(true);
        setSpeed(msg->linear.x, msg->linear.y * 5, msg->angular.z);
    }
}
