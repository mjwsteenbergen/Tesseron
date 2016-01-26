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
    int id;

    bool completed;
    bool initiased = false;

    double oneRotation = 2 * M_PI;

    double lastPosition = -100;
    double currentPosition;

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

    void setMotorName(std::string name, double radius, int i);

    void runIntoLimit();

    void initCompleted();

    void setMultiplier(double d);

    void setManualControl(bool on);

    void setOneRotation(double rad);

    bool hasCompleted();

    void limitCheck();
};


#endif //GRIPPER_DYNAMIXELMOTOR_H
