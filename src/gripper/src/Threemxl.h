//
// Created by sanderkempen on 21-1-16.
//

#ifndef GRIPPER_THREEMXL_H
#define GRIPPER_THREEMXL_H

#include <threemxl/C3mxlROS.h>
#include <threemxl/C3mxl.h>


class Threemxl {

protected:
    C3mxl *spindle;
    double startPos;
    double gotoPos = 0;
    double previous = 0;
    double rotations = 0;
    double oneRotation = 64.0;
    double prev = 0.0;
    bool manualControl = false;
    bool initialised = false;
    bool completed;

public:
    void setSpeed(double speed);

    void initialise();

    void gotoPosition(double m);

    void loopOnce();

    void stop();

    void intialiseExternal();

    void setManualControl(bool on);

    bool isInitialised();


    bool hasCompleted();
};


#endif //GRIPPER_THREEMXL_H
