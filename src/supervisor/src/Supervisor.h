//
// Created by newnottakenname on 14.12.15.
//

#ifndef SUPERVISOR_SUPERVISOR_H
#define SUPERVISOR_SUPERVISOR_H

#include "ros/ros.h"
#include "supervisor/StatusMessage.h"
#include "Tile.h"
#include "ImageReader.h"

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

    void moveTo(int x, int y);

    void moveBack(int distance);

    void dropTile(bool fully);

    void getTile(TileColor color);

    void pickupTile(bool fully);

    void readyNextTile(TileColor color);

    void getInstructions(TileColor image[MOSAIC_SIZE][MOSAIC_SIZE]);
};



#endif //SUPERVISOR_SUPERVISOR_H
