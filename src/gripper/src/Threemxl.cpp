//
// Created by sanderkempen on 21-1-16.
//

#include <rosconsole/macros_generated.h>
#include <ros/rate.h>
#include <ros/assert.h>
#include <threemxl/dxlassert.h>
#include <threemxl/dxlassert.h>
#include "threemxl/platform/io/configuration/XMLConfiguration.h"
#include "Threemxl.h"



double speedModifier = 10.0;

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

//    intialiseExternal();

    while(true)
    {
        init_rate.sleep();
        double init = spindle->presentPos();
        //if(init != 0.0){
            gotoPos = spindle->presentPos();
            startPos = spindle->presentPos();
            break;
        //}
    }

    ROS_INFO("Threemxl initialised");

}

void Threemxl::intialiseExternal()
{
    DXL_SAFE_CALL(spindle->set3MxlMode(EXTERNAL_INIT));
    DXL_SAFE_CALL(spindle->setAcceleration(2));
    DXL_SAFE_CALL(spindle->setSpeed(8));
    DXL_SAFE_CALL(spindle->setTorque(5));

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
//        {//(prev == pspeed && pspeed != 0)
        if(pstat == M3XL_STATUS_INIT_DONE )
//            if(stopped == 10)
        {
//            int i2q23 = 0;
            break;
//            }
//            else{
//                stopped++;
//            }
            //break;
        }
        else
        {
            ROS_FATAL_STREAM("Couldn't initialize arm: " << spindle->translateErrorCode(pstat2));
            ROS_INFO("Waiting for threemxl to initialize");
            prev = pspeed;
        }
        loop_rate.sleep();
    }

    DXL_SAFE_CALL(spindle->set3MxlMode(SPEED_MODE));
}

void Threemxl::setManualControl(bool on)
{
    manualControl = on;
}

void Threemxl::move(double m) {

//    int state = spindle->getState();
//
//
//    startPos = spindle->presentPos();
//
//    spindle->set3MxlMode(POSITION_MODE);
//
    //- 0.0014
    double distance = (m / (0.004 )) * 2 * M_PI + startPos;
//
//    //
//    spindle->setPos(distance, 8*M_PI, 32*M_PI, false);
//
//    double endPosRad = spindle->presentPos();

    gotoPos = distance;


}

void Threemxl::setSpeed(double speed)
{
    spindle->getState();
    if(spindle->present3MxlMode() != SPEED_MODE)
    {
        spindle->set3MxlMode(SPEED_MODE);
    }
    spindle->setSpeed(speed, false);
}

bool speeding = false;

void Threemxl::loop()
{
    spindle->getState();
    if(manualControl)
    {
        return;
    }
    double measured = spindle->presentPos();

    if(measured == 0.0)
    {
        return;
    }
    double delta = measured - previous;
    if(fabs(delta) > 40.0)
    {
        if(delta < 0)
        {
            rotations++;
        }
        else
        {
            rotations--;
        }
    }
    double current_pos = measured + oneRotation * rotations + startPos;

    double speed = current_pos - gotoPos;

    spindle->setAcceleration(5, false);

//    double pp = spindle->presentPos();
    if(spindle->present3MxlMode() != SPEED_MODE)
    {
        spindle->set3MxlMode(SPEED_MODE);
    }
    if(fabs(speed) > 0.1)
    {
        if(speed > 10) //TODO CHECK ME
        {
            spindle->setSpeed(-10, false);
        }
        else
        {
            if(speed < -10)
            {
                spindle->setSpeed(10);
            }
            else{
                spindle->setSpeed(-speed/2.0, false);
            }
        }
        speeding = true;

        std::string info = std::to_string(current_pos);
        info.append("\tGT:");
        info.append(std::to_string(gotoPos));
        info.append("\tRS:");
        info.append(std::to_string(speed));
        info.append("\tRS:");
        info.append(std::to_string(spindle->presentSpeed()));
        info.append("\tRT:");
        info.append(std::to_string(rotations));
        ROS_INFO(info.c_str());
        //}
    }
    else
    {
        spindle->setSpeed(0, false);
        speeding = false;
    }

    previous = measured;
//    spindle->setSpeed(speed / speedModifier, false);


//    ros::Rate sleep(2);
//    sleep.sleep();

}

void Threemxl::stop() {
    spindle->set3MxlMode(PWM_MODE);
    spindle->setPWM(0, false);
}