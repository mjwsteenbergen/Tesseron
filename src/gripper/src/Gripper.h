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
#include "DynamixelMotor.h"
#include "Threemxl.h"


class Gripper {

protected:
    ros::NodeHandle nh_;

    ros::ServiceServer LaydownClient;
    ros::ServiceServer PickUpClient;
    ros::ServiceServer MoveClient;

    ros::Publisher statusPublisher;

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


    void sendErrorMessage(const std::string message, const std::string sender);

    bool handleMove(gripper::MoveGripperRequest_<std::allocator<void> > &req,
                    gripper::MoveGripperResponse_<std::allocator<void> > &res);

    void loop();

    ros::ServiceClient Tilt_Command;

    void TwistCallback(const geometry_msgs::Twist::ConstPtr &msg);


    void setSpeed(double speedMX, double speedSpindle, double speedRX);

    void initialise();
};


#endif //GRIPPER_GRIPPER_H
