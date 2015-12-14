//
// Created by newnottakenname on 14.12.15.
//

#include "Supervisor.h"
#include "std_msgs/Bool.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervisor");
    Supervisor sups;
    sups.init();
}

void Supervisor::init() {
    nh_.subscribe("/basecontrol", 50, &Supervisor::nodeInitialised, this);
}

void Supervisor::nodeInitialised(StatusMessage_ message) {

}
