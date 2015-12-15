//
// Created by newnottakenname on 14.12.15.
//

#ifndef SUPERVISOR_SUPERVISOR_H
#define SUPERVISOR_SUPERVISOR_H

#include "ros/ros.h"
#include "supervisor/StatusMessage.h"

enum TileColor{ Blue = 0, Grey = 1, White = 2, Black = 3};

class Supervisor {
protected:

    ros::NodeHandle nh_;
    const static int numberOfNodes = 2;
    int numberOfBootedNodes;
    ros::Subscriber s_;
    ros::ServiceClient gripperService;
    ros::ServiceClient wheelService;
    ros::ServiceClient layDownService;

public:
    Supervisor() : nh_("~")
    {
        init();
    }
    void init();

    void nodeInitialised(const supervisor::StatusMessage::ConstPtr &message);

    void layFloor();

    void layRow();

    void moveTo(int x, int y);

    void moveBack(int distance);

    void dropTile(bool fully);

    void getTile(TileColor color);

    void pickupTile(bool fully);

    void readyNextTile(TileColor color);
};



#endif //SUPERVISOR_SUPERVISOR_H
