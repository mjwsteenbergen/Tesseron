//
// Created by newnottakenname on 14.12.15.
//

#include "Supervisor.h"
#include "supervisor/MoveGripper.h"
#include "supervisor/Wheel.h"
#include "supervisor/LayDown.h"
#include "supervisor/PickUp.h"
#include "JoyStick.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Char.h>
#include <sensor_msgs/Joy.h>


static const std::string filename = "/home/newnottakenname/Coding/Tesseron/image";


int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervisor");
    Supervisor sups;

//    sups.toManualControl();

    sups.toAutomaticControl();
    sups.layFloor();
//    sups.getTile('D');
//    sups.getTile('A');
//    sups.getTile('C');
//    sups.getTile('B');
//    sups.pickupTile(false);

    ros::Rate sleep(0.3);
    sleep.sleep();
//
//    sups.pickupTile(true);
//    sups.moveTo(-0.11, 0.173);

    //Move to laydown
//    sups.moveTo(-0.03, 0.15);
//    sups.layTile(false);

//    ros::Rate sleep2(0.3);
//    sleep2.sleep();
//    sups.pickupTile(true);

    //Move to shutdown
    sups.moveTo(-0.03, 0.05);
    sups.layTile(true);
}

void Supervisor::toManualControl() {
    ros::NodeHandle n;
    gripper_Publisher = n.advertise<geometry_msgs::Twist>("/grippercontrol", 1);
    base_Publisher = n.advertise<geometry_msgs::Twist>("/basecontrol", 1);
    joy_subscriber = nh_.subscribe("/joy", 50, &Supervisor::joystickFeedback, this);
    ros::spin();
}

void Supervisor::toAutomaticControl() {
    numberOfBootedNodes = 0;
    ROS_INFO("Looking for nodes..");
    ros::Rate r(10);
    init_subscriber = nh_.subscribe("/Tesseron/init", 50, &Supervisor::nodeInitialised, this);

    while(numberOfBootedNodes < numberOfNodes && ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("All Nodes Have Correctly Booted");

    //Create Clients (Services)

    ros::NodeHandle n;
    gripperService = n.serviceClient<supervisor::MoveGripper>("/Gripper");
    wheelService = n.serviceClient<supervisor::Wheel>("/base/Wheels");
    layDownService = n.serviceClient<supervisor::LayDown>("/laydown");
    pickUpService = n.serviceClient<supervisor::PickUp>("/pickup");

    tile_pusherService = n.advertise<std_msgs::Char>("/pushtile", 5);

    //testAllTheThings();
}

void Supervisor::testAllTheThings()
{
    ros::Rate sleep(0.3);

    sleep.sleep();

//    supervisor::MoveGripper req1;
//    gripperService.call(req1);
//
//    supervisor::Wheel req2;
//    req2.request.distance = 0.0;
//    wheelService.call(req2);
//
//    supervisor::LayDown req3;
//    layDownService.call(req3);
//
//    supervisor::PickUp req4;
//    pickUpService.call(req4);

    std_msgs::Char pub1;
    pub1.data = 'A';
    tile_pusherService.publish(pub1);

    ros::spinOnce();
    sleep.sleep();

    pub1.data = 'B';
    tile_pusherService.publish(pub1);

    ros::spinOnce();
    sleep.sleep();

    pub1.data = 'C';
    tile_pusherService.publish(pub1);

    ros::spinOnce();
    sleep.sleep();

    pub1.data = 'D';
    tile_pusherService.publish(pub1);


//    geometry_msgs::Twist pub2;
//    gripper_Publisher.publish(pub2);
//    base_Publisher.publish(pub2);
//
    ros::spin();
}

void Supervisor::getInstructions(char image[MOSAIC_SIZE][MOSAIC_SIZE]){
    ImageReader reader;
    reader.GetImage(filename, image);
}

void Supervisor::nodeInitialised(const std_msgs::Bool::ConstPtr &mes) {
    numberOfBootedNodes++;
    ROS_INFO("Node %i booted", numberOfBootedNodes);
}

void Supervisor::nodeError(const supervisor::ErrorMessage::ConstPtr &message) {
    ROS_ERROR("Node %s had a problem. Error message: %s", message.get()->sender.c_str(), message.get()->error.c_str());
}

void Supervisor::layFloor() {

    char image[MOSAIC_SIZE][MOSAIC_SIZE];
    getInstructions(image);

//    for(int i = 0; i < MOSAIC_SIZE; i++)
//    {
        //TODO add webcam
        for(int j = 0; j < MOSAIC_SIZE; j++)
        {
            getTile(image[0][j]);
            getGlue();
            moveTo(-0.03, 0.10 + j*0.025);
            layTile(false);
//            ros::Rate sleep(0.3);
//            sleep.sleep();
            readyNextTile('-');

            ros::Rate sleep(0.66);
            sleep.sleep();

            pickupTile(true);
        }
//        moveBack(3);
//    }
}

void Supervisor::moveBack(int distance) {

    supervisor::Wheel srv;
    srv.request.distance = distance;
    if (wheelService.call(srv))
    {
        if(!srv.response.succeeded)
        {
            ROS_ERROR("There was an error, while calling wheelService: %s", srv.response.error.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to call service wheelService");
    }

}

/**
 * x - spindle
 * y - MX
 */
void Supervisor::moveTo(double x, double y) {
    supervisor::MoveGripper srv;
    srv.request.x = x;
    srv.request.y = y;
    if (gripperService.call(srv))
    {
        if(!srv.response.succeeded)
        {
            ROS_ERROR("There was an error, while calling gripperService: %s", srv.response.error.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to call service wheelService");
    }
}

void Supervisor::layTile(bool b) {
    supervisor::LayDown srv;
    srv.request.fully = (unsigned char) b;
    if (layDownService.call(srv))
    {
        if(!srv.response.succeeded)
        {
            ROS_ERROR("There was an error, while calling wheelService: %s", srv.response.error.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to call service wheelService");
    }
}

void Supervisor::moveToTilePosition(char color) {

    if(color == 'A')
    {
        moveTo(-0.16, 0.13);
    }
    else if(color == 'B')
    {
        moveTo(-0.16, 0.17);
    }
    else if(color == 'C')
    {
        moveTo(-0.16, 0.333);
    }
    else if(color == 'D')
    {
        moveTo(-0.16, 0.372);
    }
    else {
        ROS_ERROR("There is no color like this. Please check color spectrum");
    }

    //pickupTile(false);

    //readyNextTile(color);

}

void Supervisor::moveBackFrom(char color)
{
    if(color == 'A')
    {
        moveTo(-0.11, 0.13);
    }
    else if(color == 'B')
    {
        moveTo(-0.11, 0.17);
    }
    else if(color == 'C')
    {
        moveTo(-0.11, 0.333);
    }
    else if(color == 'D')
    {
        moveTo(-0.11, 0.372);
    }
    else {
        ROS_ERROR("There is no color like this. Please check color spectrum");
    }

}

void Supervisor::getGlue()
{
//    5.84
    //8.02
    moveTo(-0.16, 0.17 + 0.0802);
}

void Supervisor::getTile(char i) {
    moveToTilePosition(i);
    pickupTile(false);
    //ros::Rate sleep(0.3);
    //sleep.sleep();
    readyNextTile('+');

    ros::Rate sleep(0.66);
    sleep.sleep();

    pickupTile(true);
    //moveBackFrom(i);
    readyNextTile(i);
}

void Supervisor::pickupTile(bool fully) {
    supervisor::PickUp srv;
    srv.request.fully = (unsigned char) fully;
    if (pickUpService.call(srv))
    {
        if(!srv.response.succeeded)
        {
//            ROS_ERROR("There was an error, while calling wheelService: %s", srv.response.error);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service wheelService");
    }
}

void Supervisor::readyNextTile(char colour) {
    if(colour == 'A' || colour == 'B' || colour == 'C' || colour == 'D' || colour == '+' || colour == '-'){
        std_msgs::Char msg;
        msg.data = (unsigned char) colour;
        tile_pusherService.publish(msg);
    }
    else
    {
        ROS_ERROR("Not a valid char!");
    }


}

void Supervisor::manualControl(double baseL, double baseR, double spindle, double dynamixelBar, double dynamixelPlacer)
{
    geometry_msgs::Twist msg;
    msg.linear.x = dynamixelBar;
    msg.linear.y = spindle;
    msg.angular.z = dynamixelPlacer;

    gripper_Publisher.publish(msg);

    geometry_msgs::Twist msg2;
    msg2.linear.x = baseL;
    msg2.linear.y = baseR;
    base_Publisher.publish(msg2);
}

bool stop = true;

void Supervisor::joystickFeedback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->axes[23] != 0.0 | joy->axes[24] != 0.0 | joy->axes[25] != 0.0 )
    {
        //return;
    }
    if(joy->buttons[PS3_BUTTON_START] > 0.001)
    {
        stop = false;
    }

    if(joy->buttons[PS3_BUTTON_ACTION_CROSS] > 0.0001 | stop)
    {
        manualControl(0,0,0,0,0);
        stop = true;
        return;
    }

    double base = joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
    double baseL = 0;
    double baseR = 0;
    double joyLeft = joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
    double dynaBar = joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
    double spindle = joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
    double down = static_cast<double>(joy->axes[13]);
    double up = static_cast<double>(joy->axes[15]);
    if(joyLeft > 0)
    {
        baseL = joyLeft;
    }
    else{
        baseR = -1 * joyLeft;
    }

    if(down > 0.001 && up > 0.001)
    {
        up = 0;
        down = 0;
    }

    manualControl(baseL + base, baseR + base, spindle, up - down, dynaBar);
}



