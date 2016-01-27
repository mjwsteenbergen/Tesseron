//
// Created by newnottakenname on 27.01.16.
//

#include <rosconsole/macros_generated.h>
#include <ros/console.h>
#include <threemxl/platform/hardware/dynamixel/CDxlCom.h>
#include <ros/init.h>
#include <math.h>
#include <threemxl/platform/hardware/dynamixel/3mxl/3mxlControlTable.h>
#include <threemxl/platform/hardware/dynamixel/CDxlConfig.h>
#include <threemxl/C3mxlROS.h>
#include <threemxl/platform/io/configuration/XMLConfiguration.h>
#include "Glue.h"

void Glue::init() {
    ros::Rate init_rate(1);

    CXMLConfiguration configuration;
    ROS_ASSERT(configuration.loadFile("/home/newnottakenname/Coding/Tesseron/src/gripper/cfg/solenoid.xml"));

    std::string name = "motor_comm"; //FIXME

    solenoid = new C3mxlROS(name.c_str());

    CDxlConfig spindleConfig;
    spindleConfig.readConfig(configuration.root().section("left"));

    solenoid->setConfig(&spindleConfig);

    double spoed = 5.988408933E-3;

    solenoid->setWheelDiameter(spoed/(M_PI*2));


//    solenoid->set3MxlMode(SPEED_MODE);
//    solenoid->getState();

    while (ros::ok() && solenoid->init() != DXL_SUCCESS)
    {
        ROS_WARN("Couldn't connect to solenoid, will continue trying every second - Spindle");
        init_rate.sleep();
    }



    init_rate.sleep();

    ROS_INFO("Solenoid initialised");
}

void Glue::on() {
    if(solenoid->present3MxlMode() != PWM_MODE)
    {
        solenoid->set3MxlMode(PWM_MODE);
    }

    solenoid->setPWM(1, false);

}

void Glue::off() {
    if(solenoid->present3MxlMode() != PWM_MODE)
    {
        solenoid->set3MxlMode(PWM_MODE);
    }

    solenoid->setPWM(-1, false);

}

void Glue::detach() {
    if(solenoid->present3MxlMode() != PWM_MODE)
    {
        solenoid->set3MxlMode(PWM_MODE);
    }

    solenoid->setPWM(0, false);
}
