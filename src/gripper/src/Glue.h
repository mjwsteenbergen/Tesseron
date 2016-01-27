//
// Created by newnottakenname on 27.01.16.
//

#ifndef GRIPPER_GLUE_H
#define GRIPPER_GLUE_H


class Glue {
private:
    C3mxl *solenoid;
public:
    void init();
    void on();
    void off();
    void detach();
};


#endif //GRIPPER_GLUE_H
