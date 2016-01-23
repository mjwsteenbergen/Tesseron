//
// Created by sanderkempen on 21-1-16.
//

#ifndef GRIPPER_THREEMXL_H
#define GRIPPER_THREEMXL_H

#include <threemxl/C3mxlROS.h>


class Threemxl {

protected:
    C3mxl *spindle;
    double startPos;
public:
    void setSpeed(double speed);

    void initialise();

    void move(double m);

    void loop();

    void stop();

    void intialiseExternal();
};


#endif //GRIPPER_THREEMXL_H
