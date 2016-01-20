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

    double lastPosition;
    double wantedPosition;
    double startPosition;
    double radius;
    double speedModifier = 5;
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
};


#endif //GRIPPER_DYNAMIXELMOTOR_H
