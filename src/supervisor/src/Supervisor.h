//
// Created by newnottakenname on 14.12.15.
//

#ifndef SUPERVISOR_SUPERVISOR_H
#define SUPERVISOR_SUPERVISOR_H

#include "ros/ros.h"

class Supervisor {
protected:
    ros::NodeHandle nh_;
    ros::Subscriber subs;
public:
    void init();

    void nodeInitialised(StatusMessage message);
};


#endif //SUPERVISOR_SUPERVISOR_H
