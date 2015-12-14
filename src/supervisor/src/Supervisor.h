//
// Created by newnottakenname on 14.12.15.
//

#ifndef SUPERVISOR_SUPERVISOR_H
#define SUPERVISOR_SUPERVISOR_H

#include "ros/ros.h"
#include "supervisor/StatusMessage.h"

class Supervisor {
protected:

    ros::NodeHandle nh_;
    const static int numberOfNodes = 2;
    int numberOfBootedNodes;
    ros::Subscriber s_;

public:
    Supervisor() : nh_("~")
    {
        init();
    }
    void init();

    void nodeInitialised(const supervisor::StatusMessage::ConstPtr &message);
};


#endif //SUPERVISOR_SUPERVISOR_H
