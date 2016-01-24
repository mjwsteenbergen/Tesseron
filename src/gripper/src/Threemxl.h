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
    bool manualControl = false;

public:
    void setSpeed(double speed);

    void initialise();

    void move(double m);

    void loop();

    void stop();

    void intialiseExternal();

    void setManualControl(bool on);
};


#endif //GRIPPER_THREEMXL_H
