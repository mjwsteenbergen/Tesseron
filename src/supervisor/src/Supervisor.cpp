//
// Created by newnottakenname on 14.12.15.
//

#include "Supervisor.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervisor");
    Supervisor sups;
    sups.init();
}

void Supervisor::init() {
    nh_.subscribe("/Tesseron/init", 50, &Supervisor::nodeInitialised, this);
    numberOfBootedNodes = 0;
    ROS_INFO("Looking for nodes..");
    while(numberOfBootedNodes != numberOfNodes)
    {
        ros::spinOnce();
    }
    ROS_INFO("All Nodes Have Correctly Booted");
}

void Supervisor::nodeInitialised(const supervisor::StatusMessage::ConstPtr &message) {
    if(message.get()->completed == false)
    {
        ROS_ERROR("A node could not boot correctly. We will not continue %s", message.get()->error.c_str());
        ros::shutdown();
    }
    else
    {
        numberOfBootedNodes++;
        ROS_INFO("Node %i booted", numberOfBootedNodes);
    }
}
