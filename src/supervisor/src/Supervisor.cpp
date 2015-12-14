//
// Created by newnottakenname on 14.12.15.
//

#include "Supervisor.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervisor");
    Supervisor sups;
}

void Supervisor::init() {
    numberOfBootedNodes = 0;
    ROS_INFO("Looking for nodes..");
    ros::Rate r(10);
    s_ = nh_.subscribe("/Tesseron/init", 50, &Supervisor::nodeInitialised, this);

    while(numberOfBootedNodes != numberOfNodes && ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("All Nodes Have Correctly Booted");
}

void Supervisor::nodeInitialised(const supervisor::StatusMessage::ConstPtr &message) {
    if(message.get()->completed == false)
    {
        ROS_ERROR("Node %s could not boot correctly. Error message: %s", message.get()->sender.c_str(), message.get()->error.c_str());
    }
    else
    {
        numberOfBootedNodes++;
        ROS_INFO("Node %i booted", numberOfBootedNodes);
    }
}
