//
// Created by newnottakenname on 14.12.15.
//

#include "Supervisor.h"
#include "supervisor/Gripper.h"
#include "supervisor/Wheel.h"
#include "supervisor/LayDown.h"
#include "supervisor/PickUp.h"

static const std::string filename = "/home/newnottakenname/Coding/Tesseron/image";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervisor");
    Supervisor sups;
    sups.layFloor();

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

    //Create Clients (Services)

    ros::NodeHandle n;
    gripperService = n.serviceClient<supervisor::Gripper>("Gripper");
    wheelService = n.serviceClient<supervisor::Wheel>("Wheel");
    layDownService = n.serviceClient<supervisor::LayDown>("LayDown");
}

void Supervisor::getInstructions(TileColor image[MOSAIC_SIZE][MOSAIC_SIZE]){
    ImageReader reader;
    reader.GetImage(filename, image);
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

void Supervisor::layFloor() {

    TileColor image[MOSAIC_SIZE][MOSAIC_SIZE];
    getInstructions(image);

    for(int i = 0; i < MOSAIC_SIZE; i++)
    {
        //TODO add webcam
        for(int j = 0; j < MOSAIC_SIZE; j++)
        {
            getTile(image[i][j]);
            moveTo(i,j);
            dropTile(true);
        }
        moveBack(3);
    }
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

void Supervisor::moveTo(int x, int y) {
    supervisor::Gripper srv;
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

void Supervisor::dropTile(bool b) {
    supervisor::LayDown srv;
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

void Supervisor::getTile(TileColor color) {

    int defaultx = 0;
    int defaulty = 0;
    switch (color)
    {
        case Blue:
            moveTo(defaultx, defaulty); // Move to pickup point //TODO
            break;
        default:
            ROS_ERROR("There is no color like this. Please check color spectrum");
            break;
    }

    pickupTile(false);

    readyNextTile(color);

}

void Supervisor::pickupTile(bool fully) {
    supervisor::PickUp srv;
    srv.request.fully = (unsigned char) fully;
    if (layDownService.call(srv))
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

void Supervisor::readyNextTile(TileColor color) {
    //TODO maybe add service?

}
