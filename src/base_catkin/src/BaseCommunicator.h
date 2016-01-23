//
// Created by newnottakenname on 14.12.15.
//

#ifndef BASE_CATKIN_BASECOMMUNICATOR_H
#define BASE_CATKIN_BASECOMMUNICATOR_H


class BaseCommunicator {
public:
    long Spin();

    void init();

protected:
    Base base;
    ros::Subscriber vel_sub_;
    ros::NodeHandle	nh_;
    ros::Publisher	status_pub_;


};


#endif //BASE_CATKIN_BASECOMMUNICATOR_H
