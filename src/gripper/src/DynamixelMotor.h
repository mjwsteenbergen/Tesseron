//
// Created by newnottakenname on 15.01.16.
//

#ifndef GRIPPER_DYNAMIXELMOTOR_H
#define GRIPPER_DYNAMIXELMOTOR_H

#include <dynamixel_msgs/JointState.h>


class DynamixelMotor {

private:
    ros::NodeHandle nh_;

    ros::Publisher MX_Pub;
    ros::Subscriber MX_Sub;

    void init();

    int rotations;

    bool completed;

    double oneRotation = 2 * M_PI;
    double lastPosition;
    double wantedPosition;
    double startPosition;
    double radius;
    double speedModifier = 5;
    bool manualControl = false;
    double minus;

public:

    DynamixelMotor() : nh_("~"){
        init();
    }

    void updateSpeed();

    void positionCallback(const dynamixel_msgs::JointState::ConstPtr &mes);

    void setSpeed(double d);

    void gotoPosition(double position);

    void loop();

    void loopOnce();

    void setMotorName(std::string name, double radius);

    void runIntoLimit();

    void initCompleted();

    void setMultiplier(double d);

    void setMinus(double d);

    void setManualControl(bool on);

    void setOneRotation(double rad);

    bool hasCompleted();
};


#endif //GRIPPER_DYNAMIXELMOTOR_H
