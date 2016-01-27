//
// Created by newnottakenname on 14.12.15.
//

#ifndef SUPERVISOR_SUPERVISOR_H
#define SUPERVISOR_SUPERVISOR_H

#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include "ros/ros.h"
#include "supervisor/ErrorMessage.h"
#include "ImageReader.h"

class Supervisor {
protected:

    ros::NodeHandle nh_;
    const static int numberOfNodes = 2;
    int numberOfBootedNodes;

    ros::Subscriber joy_subscriber;
    ros::Subscriber init_subscriber;

    ros::ServiceClient gripperService;
    ros::ServiceClient wheelService;
    ros::ServiceClient layDownService;
    ros::ServiceClient pickUpService;
    ros::ServiceClient glueService;


    ros::Publisher tile_pusherService;
    ros::Publisher gripper_Publisher;
    ros::Publisher base_Publisher;



public:
    Supervisor() : nh_("~")
    {
    }


    void layFloor();

    void moveTo(double x, double y);

    void moveBack(float distance);

    void layTile(bool fully);

    void moveToTilePosition(char color);

    void pickupTile(bool fully);

    void readyNextTile(char colour);

    void getInstructions(char image[MOSAIC_SIZE][MOSAIC_SIZE]);

    void manualControl(double baseL, double baseR, double spindle, double dynamixelBar, double dynamixelPlacer);


    void joystickFeedback(const sensor_msgs::Joy_<std::allocator<void> >::ConstPtr &joy);

    void nodeError(const supervisor::ErrorMessage::ConstPtr &message);

    void nodeInitialised(const std_msgs::Bool::ConstPtr &mes);

    void testAllTheThings();

    void toManualControl();

    void toAutomaticControl();

    void moveBackFrom(char color);

    void getTile(char i);

    void getGlue();

    void sprayAllTheThings();
};



#endif //SUPERVISOR_SUPERVISOR_H
