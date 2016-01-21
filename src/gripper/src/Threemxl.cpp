//
// Created by sanderkempen on 21-1-16.
//

#include <rosconsole/macros_generated.h>
#include <ros/rate.h>
#include <ros/assert.h>
#include <threemxl/dxlassert.h>
#include <threemxl/C3mxlROS.h>
#include <threemxl/dxlassert.h>
#include "threemxl/platform/io/configuration/XMLConfiguration.h"
#include "Threemxl.h"

void Threemxl::initialise() {
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
        ROS_WARN("Couldn't connect to spindle, will continue trying every second - Spindle");
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

    ROS_INFO("Threemxl initialised");

}

void Threemxl::move(double m) {

    int state = spindle->getState();


    startPos = spindle->presentPos();

    spindle->set3MxlMode(POSITION_MODE);

    double speed = (m / 0.004) * 2 * M_PI;

    //
    spindle->setPos(startPos + speed, 8*M_PI, 32*M_PI, false);

    double endPosRad = spindle->presentPos();


}

void Threemxl::setSpeed(double speed)
{
    spindle->getState();
    spindle->set3MxlMode(SPEED_MODE);
    spindle->setSpeed(speed);
}

void Threemxl::loop()
{
    spindle->getState();
}

void Threemxl::stop() {
    spindle->set3MxlMode(PWM_MODE);
    spindle->setPWM(0, false);
}