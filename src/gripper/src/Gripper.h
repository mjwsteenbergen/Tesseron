//
// Created by newnottakenname on 15.12.15.
//

#ifndef GRIPPER_GRIPPER_H
#define GRIPPER_GRIPPER_H

#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <threemxl/dxlassert.h>
#include <gripper/LayDownRequest.h>
#include <gripper/LayDownResponse.h>
#include <gripper/PickUpRequest.h>
#include <gripper/PickUpResponse.h>
#include <gripper/MoveGripperRequest.h>
#include <gripper/MoveGripperResponse.h>
#include <geometry_msgs/Twist.h>
#include "DynamixelMotor.h"


class Gripper {

protected:
    ros::NodeHandle nh_;

    ros::ServiceServer LaydownClient;
    ros::ServiceServer PickUpClient;
    ros::ServiceServer MoveClient;

    ros::Publisher statusPublisher;

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

    void initialiseSpindle();

    void initialiseGearBar();

    C3mxl *spindle;

    void sendErrorMessage(const std::string message, const std::string sender);

    bool resetGearBar();

    bool resetSpindle();

    bool handleMove(gripper::MoveGripperRequest_<std::allocator<void> > &req,
                    gripper::MoveGripperResponse_<std::allocator<void> > &res);

    void loop();

    void STOP();

    ros::ServiceClient Tilt_Command;

    void handleMoveSpindle(gripper::MoveGripperRequest_<std::allocator<void>> &req,
                           gripper::MoveGripperResponse_<std::allocator<void>> &res);

    void handleMoveMX(gripper::MoveGripperRequest_<std::allocator<void>> &request_,
                      gripper::MoveGripperResponse_<std::allocator<void>> &response_);

    void TwistCallback(const geometry_msgs::Twist::ConstPtr &msg);


    void setSpeed(double speedMX, double speedSpindle, double speedRX);
};


#endif //GRIPPER_GRIPPER_H
