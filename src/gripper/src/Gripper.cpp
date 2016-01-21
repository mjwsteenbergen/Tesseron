//
// Created by newnottakenname on 15.12.15.
//

#include "Gripper.h"
#include "gripper/LayDown.h"
#include "gripper/PickUp.h"
#include "gripper/MoveGripper.h"
#include "gripper/StatusMessage.h"
#include "threemxl/platform/io/configuration/XMLConfiguration.h"
#include "dynamixel_controllers/StartController.h"
#include "dynamixel_controllers/SetSpeed.h"
#include "std_msgs/Float64.h"
#include "DynamixelMotor.h"
#include <iostream>
#include <string>
#include <math.h>
#include <thread>

std_msgs::Float64 pubmesh;

ros::Subscriber PS3_Sub;

ros::Publisher RX_Pub;

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

    PS3_Sub   = nh_.subscribe("/basecontrol", 50, &Gripper::TwistCallback, this);
    Tilt_Command = nh_.serviceClient<dynamixel_controllers::SetSpeed>("/tilt_controller/set_speed");


    //StatusPublisher
    statusPublisher = nh_.advertise<gripper::StatusMessage>("Tesseron/init", 10);






//    initialiseSpindle();
    initialiseGearBar();

    //resetArms();
}

void Gripper::initialiseSpindle() {
    ros::Rate init_rate(1);

    CXMLConfiguration configuration;
    ROS_ASSERT(configuration.loadFile("/home/newnottakenname/Coding/Tesseron/src/gripper/cfg/gripper.xml"));

    std::string name = "motor_comm"; //FIXME

    spindle = new C3mxlROS(name.c_str());

    CDxlConfig spindleConfig;
    spindleConfig.readConfig(configuration.root().section("left"));

    spindle->setConfig(&spindleConfig);

    double spoed = 5.988408933E-3;

    spindle->setWheelDiameter(spoed/(M_PI*2));


//    spindle->set3MxlMode(SPEED_MODE);
//    spindle->getState();

    while (ros::ok() && spindle->init() != DXL_SUCCESS)
    {
        sendErrorMessage("Couldn't connect to spindle, will continue trying every second", "Spindle");
        init_rate.sleep();
    }

    DXL_SAFE_CALL(spindle->set3MxlMode(EXTERNAL_INIT));
    DXL_SAFE_CALL(spindle->setAcceleration(2));
    DXL_SAFE_CALL(spindle->setSpeed(8));
    DXL_SAFE_CALL(spindle->setTorque(3));

    ros::Rate first_sleep(1.0/2.0);
    first_sleep.sleep();

    ros::Rate loop_rate(10);
    double prev = 0.0;
    while(ros::ok())
    {
        spindle->getLog();
        int pstat = spindle->presentStatus();
        int pstat2 = spindle->present3MxlMode();

        int i = spindle->get3MxlMode();
        double pspeed = spindle->presentPos();
        double ls = spindle->getLinearPos();
        double ls2 = spindle->getPos();

//        if (pstat == M3XL_STATUS_INIT_DONE ||  pspeed == 0)
//        {
            if(prev == pspeed && pspeed != 0)
//            if(stopped == 10)
            {
                int i2q23 = 0;
                break;
//            }
//            else{
//                stopped++;
//            }
            //break;
        }
        else
        {
            ROS_INFO("Waiting for dynamixel to initialize");
            prev = pspeed;
        }
        loop_rate.sleep();

    }

    ROS_INFO("Spindle initialised");



}

void Gripper::initialiseGearBar() {
//    DYNAMIXEL_CONTROLLERS_MESSAGE_SETSPEED_H::dynamixel_controllers::StartController startController;

//    dynamixel_controllers::SetSpeedRequest req;
//    req.speed = 1.0;
//
//    dynamixel_controllers::SetSpeedResponse resp;
//
//
//    Tilt_Command.call(req, resp);





    MX.setMotorName("/pan_controller/", 3E-02);
    MX.runIntoLimit();
    RX.setMinus(2.62);
    RX.setMotorName("/tilt_controller/", 16E-3);
    RX.initCompleted();




    //ros::Rate loop(1);

    //loop.sleep();

//    std_msgs::Float64 mes2;

//    mes2.data = 0;

//    MX_Pub.publish(mes2);

    ROS_INFO("Dynamixel initialised");
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
    res.succeeded = (unsigned char) true;

    handleMoveSpindle(req, res);
    MX.gotoPosition(req.x);


    return true;
}

void Gripper::handleMoveSpindle(gripper::MoveGripperRequest_<std::allocator<void> > &req,
                         gripper::MoveGripperResponse_<std::allocator<void> > &res) {

    int state = spindle->getState();


    startPosRad = spindle->presentPos();

    spindle->set3MxlMode(POSITION_MODE);

    double speed = (req.y / 0.004) * 2 * M_PI;

    //
    spindle->setPos(startPosRad + speed, 8*M_PI, 32*M_PI, false);

    double endPosRad = spindle->presentPos();


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

void Gripper::setSpeed(double speedMX, double speedSpindle, double speedRX)
{
    spindle->getState();
    spindle->set3MxlMode(SPEED_MODE);
    spindle->setSpeed(speedSpindle);

    RX.setSpeed(speedMX);
    MX.setSpeed(speedRX);
}

void Gripper::loop()
{
    ros::Rate loop_rate(1000);
    while(ros::ok()){
//        spindle->getState();


        MX.loopOnce();
        RX.loopOnce();

        //double d = (2*M_PI + 0.4*M_PI);
        //int i = (int) d;

        //std::this_thread::sleep_for(std::chrono::microseconds(1));

//        ros::Rate waitTime(0.5);
//        waitTime.sleep();
//
        loop_rate.sleep();
//
//        mes.data = 0;
//
//        MX_Pub.publish(mes);
//
//        loop_rate.sleep();
//        loop_rate.sleep();
//        loop_rate.sleep();


//        int i = __cplusplus;

        //double posd = spindle->presentPos();
        //int pos = spindle->getLinearPos();


        //std::string s= std::to_string(spindle->getPos());
//        ROS_INFO();

        ros::spinOnce();

    }

    //STOP();
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

void Gripper::STOP() {
    spindle->set3MxlMode(PWM_MODE);
    spindle->setPWM(0, false);
}


int main(int argc, char **argv)
{
    ROS_WARN("BOOTING...");
    ros::init(argc, argv, "Gripper");
    Gripper grip;
    //grip.RX.gotoPosition(0.13);
    grip.loop();
//    gripper::MoveGripperRequest req;

}

void Gripper::TwistCallback(const geometry_msgs::Twist::ConstPtr &msg) {

    setSpeed(msg->linear.x, msg->linear.y, msg->angular.z);



}
