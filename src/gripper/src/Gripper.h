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

class Gripper {

protected:
    ros::NodeHandle nh_;

    ros::ServiceServer LaydownClient;
    ros::ServiceServer PickUpClient;
    ros::ServiceServer MoveClient;

    ros::Publisher statusPublisher;



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
};


#endif //GRIPPER_GRIPPER_H
