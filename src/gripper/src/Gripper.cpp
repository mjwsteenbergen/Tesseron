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
#include "DynamixelMotor.h"
#include <iostream>
#include <string>
#include <math.h>
#include <thread>

void Gripper::sendErrorMessage(const std::string message_body, const std::string sender) {
    ROS_WARN(message_body.c_str(), "Gripper");
    gripper::StatusMessage message;
    message.completed = (unsigned char) false;
    message.error = message_body;
    message.sender = sender;
    statusPublisher.publish(message);
}


void Gripper::init()
{
    LaydownClient = nh_.advertiseService("laydown", &Gripper::handleLayDown, this);
    PickUpClient  = nh_.advertiseService("pickup" , &Gripper::handlePickUp, this );
    MoveClient    = nh_.advertiseService("move" ,   &Gripper::handleMove, this );

    speedTopic = nh_.subscribe("/grippercontrol", 50, &Gripper::TwistCallback, this);

    //StatusPublisher
    statusPublisher = nh_.advertise<gripper::StatusMessage>("Tesseron/init", 10);

    initialise();

    //resetArms();
}



void Gripper::initialise() {
    spindle.initialise();
    MX.setMotorName("/pan_controller/", 3E-02);
    MX.runIntoLimit();
    RX.setMinus(2.62);
    RX.setMotorName("/tilt_controller/", 16E-3);
    RX.initCompleted();

    ROS_INFO("Dynamixel initialised");
}

bool Gripper::handleLayDown(gripper::LayDown::Request &req, gripper::LayDown::Response &res)
{
    //TODO

    res.succeeded = (unsigned char) false;
    return true;
}

bool Gripper::handlePickUp(gripper::PickUp::Request &req, gripper::PickUp::Response &res)
{
    //TODO

    res.succeeded = (unsigned char) false;
    return true;
}

bool Gripper::handleMove(gripper::MoveGripper::Request &req, gripper::MoveGripper::Response &res)
{
    res.succeeded = (unsigned char) true;

    spindle.move(req.y);
    MX.gotoPosition(req.x);


    return true;
}





void Gripper::resetArms() {
    //TODO

    if(resetGearBar() && resetSpindle()) { //Something went wrong
        gripper::StatusMessage message;
        message.completed = (unsigned char) true;
        message.error = "";
        message.sender = "Gripper";
        statusPublisher.publish(message);
    }
    sendErrorMessage("Not Implemented", "Gripper");
}

void Gripper::setSpeed(double speedMX, double speedSpindle, double speedRX)
{
    spindle.setSpeed(speedSpindle);
    RX.setSpeed(speedMX);
    MX.setSpeed(speedRX);
}

void Gripper::loop()
{
    ros::Rate loop_rate(1000);
    while(ros::ok()){
//


        MX.loopOnce();
        RX.loopOnce();
        spindle.loop();

        //double d = (2*M_PI + 0.4*M_PI);
        //int i = (int) d;

        //std::this_thread::sleep_for(std::chrono::microseconds(1));

//        ros::Rate waitTime(0.5);
//        waitTime.sleep();
//
        loop_rate.sleep();
//
//        mes.data = 0;
//
//        MX_Pub.publish(mes);
//
//        loop_rate.sleep();
//        loop_rate.sleep();
//        loop_rate.sleep();


//        int i = __cplusplus;

        //double posd = spindle->presentPos();
        //int pos = spindle->getLinearPos();


        //std::string s= std::to_string(spindle->getPos());
//        ROS_INFO();

        ros::spinOnce();

    }

    //STOP();
}

int main(int argc, char **argv)
{
    ROS_INFO("BOOTING...");
    ros::init(argc, argv, "Gripper");
    Gripper grip;
    grip.RX.gotoPosition(0.13);
    grip.loop();
    gripper::MoveGripperRequest req;

}

void Gripper::TwistCallback(const geometry_msgs::Twist::ConstPtr &msg) {

    setSpeed(msg->linear.x, msg->linear.y, msg->angular.z);



}
