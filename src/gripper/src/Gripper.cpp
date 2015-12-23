//
// Created by newnottakenname on 15.12.15.
//

#include "Gripper.h"
#include "gripper/LayDown.h"
#include "gripper/PickUp.h"
#include "gripper/MoveGripper.h"
#include "gripper/StatusMessage.h"
#include "threemxl/platform/io/configuration/XMLConfiguration.h"

void Gripper::sendErrorMessage(const std::string message_body, const std::string sender) {
    ROS_WARN(message_body.c_str(), "Gripper");
    gripper::StatusMessage message;
    message.completed = (unsigned char) false;
    message.error = message_body;
    message.sender = sender;
    statusPublisher.publish(message);
}


void Gripper::init()
{
    LaydownClient = nh_.advertiseService("laydown", &Gripper::handleLayDown, this);
    PickUpClient  = nh_.advertiseService("pickup" , &Gripper::handlePickUp, this );
    MoveClient    = nh_.advertiseService("move" ,   &Gripper::handleMove, this );

    statusPublisher = nh_.advertise<gripper::StatusMessage>("Tesseron/init", 10);

    initialiseSpindle();
    initialiseGearBar();

    //resetArms();
}

void Gripper::initialiseSpindle() {
    ros::Rate init_rate(1);

    CXMLConfiguration configuration;
    ROS_ASSERT(configuration.loadFile("/home/newnottakenname/Coding/Tesseron/src/gripper/config/gripper.xml"));

    std::string name = "motor_comm"; //FIXME

    spindle = new C3mxlROS(name.c_str());
//    spindle->set3MxlMode(PWM_MODE);
//    spindle->setGearboxRatioMotor(3.0);
    CDxlConfig spindleConfig;
    spindleConfig.readConfig(configuration.root().section("left"));

    spindle->setConfig(&spindleConfig);
    spindle->set3MxlMode(SPEED_MODE);
    spindle->getState();

    while (ros::ok() && spindle->init() != DXL_SUCCESS)
    {
        sendErrorMessage("Couldn't connect to spindle, will continue trying every second", "Spindle");
        init_rate.sleep();
    }

    sendErrorMessage("Yolo","swag");

}

void Gripper::initialiseGearBar() {


}

bool Gripper::handleLayDown(gripper::LayDown::Request &req, gripper::LayDown::Response &res)
{
    //TODO

    res.succeeded = (unsigned char) false;
    return true;
}

bool Gripper::handlePickUp(gripper::PickUp::Request &req, gripper::PickUp::Response &res)
{
    //TODO

    res.succeeded = (unsigned char) false;
    return true;
}

bool Gripper::handleMove(gripper::MoveGripper::Request &req, gripper::MoveGripper::Response &res)
{
    //TODO

//    spindle->set3MxlMode(PWM_MODE);
//    spindle->setPWM(0.1, true);

    int state = spindle->getState();
    spindle->setSpeed(5);

    res.succeeded = (unsigned char) false;

    sendErrorMessage("no","no");

    return true;
}



void Gripper::resetArms() {
    //TODO

    if(resetGearBar() && resetSpindle()) { //Something went wrong
        gripper::StatusMessage message;
        message.completed = (unsigned char) true;
        message.error = "";
        message.sender = "Gripper";
        statusPublisher.publish(message);
    }
    sendErrorMessage("Not Implemented", "Gripper");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Gripper");
    Gripper grip;
    gripper::MoveGripperRequest req;
    gripper::MoveGripperResponse resp;
    grip.handleMove(req, resp);
    ros::spin();
}

bool Gripper::resetGearBar() {
    return false;
}

bool Gripper::resetSpindle() {
    ros::Rate loop_rate(1);
    int estat = -1;

    //FIXME

    DXL_SAFE_CALL(spindle->set3MxlMode(EXTERNAL_INIT));
    DXL_SAFE_CALL(spindle->setAcceleration(1));
    DXL_SAFE_CALL(spindle->setSpeed(-0.3));
    DXL_SAFE_CALL(spindle->setTorque(-0.1));
    while (ros::ok())
    {
        DXL_SAFE_CALL(spindle->getStatus());

        estat = spindle->presentStatus();

        if (estat != M3XL_STATUS_INITIALIZE_BUSY)
        {
            break;
        }

        loop_rate.sleep();
    }
    if (estat != M3XL_STATUS_INIT_DONE)
    {

        std::string error =  "Couldn't initialize wrist roll: ";
        error.append(spindle->translateErrorCode(estat));
//        ROS_FATAL_STREAM("Couldn't initialize wrist roll: " << spindle->translateErrorCode(estat));
        sendErrorMessage(error, "spindle");
        ROS_ISSUE_BREAK();
        return false;
    }
    return true;
}
