//
// Created by newnottakenname on 14.12.15.
//

#ifndef SUPERVISOR_SUPERVISOR_H
#define SUPERVISOR_SUPERVISOR_H

#include <sensor_msgs/Joy.h>
#include "ros/ros.h"
#include "supervisor/StatusMessage.h"
#include "Tile.h"
#include "ImageReader.h"

class Supervisor {
protected:

    ros::NodeHandle nh_;
    const static int numberOfNodes = 2;
    int numberOfBootedNodes;
    
    ros::Subscriber joy_subscriber;

    ros::ServiceClient gripperService;
    ros::ServiceClient wheelService;
    ros::ServiceClient layDownService;

    ros::Publisher gripper_Publisher;
    ros::Publisher base_Publisher;



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

    void manualControl(double baseL, double baseR, double spindle, double dynamixelBar, double dynamixelPlacer);


    void joystickFeedback(const sensor_msgs::Joy_<std::allocator<void> >::ConstPtr &joy);
};



#endif //SUPERVISOR_SUPERVISOR_H
