//
// Created by newnottakenname on 15.12.15.
//

#ifndef GRIPPER_GRIPPER_H
#define GRIPPER_GRIPPER_H

#include <ros/ros.h>
#include <gripper/LayDownRequest.h>
#include <gripper/LayDownResponse.h>
#include <gripper/PickUpRequest.h>
#include <gripper/PickUpResponse.h>
#include <gripper/MoveGripperRequest.h>
#include <gripper/MoveGripperResponse.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>
#include <std_srvs/Trigger.h>
#include "DynamixelMotor.h"
#include "Threemxl.h"
#include "Glue.h"


class Gripper {

protected:

    ros::NodeHandle nh_;

    ros::ServiceServer LaydownClient;
    ros::ServiceServer PickUpClient;
    ros::ServiceServer MoveClient;
    ros::ServiceServer GlueClient;

    ros::Publisher initPublisher;

    ros::Subscriber speedTopic;


    double startPosRad;

    void resetArms();

    bool handleLayDown(gripper::LayDownRequest_<std::allocator<void> > &req,
                       gripper::LayDownResponse_<std::allocator<void> > &res);

    bool handlePickUp(gripper::PickUpRequest_<std::allocator<void> > &req,
                      gripper::PickUpResponse_<std::allocator<void> > &res);

    void init();

public:
    Gripper() : nh_("~"){
        init();
    }


    DynamixelMotor MX;
    DynamixelMotor RX;
    Threemxl spindle;
    Glue glue;


    void sendErrorMessage(const std::string message, const std::string sender);

    bool handleMove(gripper::MoveGripperRequest_<std::allocator<void> > &req,
                    gripper::MoveGripperResponse_<std::allocator<void> > &res);

    void loop();

    ros::ServiceClient Tilt_Command;

    void TwistCallback(const geometry_msgs::Twist::ConstPtr &msg);


    void setSpeed(double speedMX, double speedSpindle, double speedRX);

    bool initialise();

    void handlePickup(bool fully);

    void handleLayDown(bool fully);

    void loopOnce();

    void moveToShutdown();

    void BlockingRX(double distance);

    void BlockingMX(double distance);

    void BlockingSpindle(double distance);

    void sprayGlue();

    bool handleMove(double spindlePos, double MXPos);

    bool handleGlue(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};


#endif //GRIPPER_GRIPPER_H
