//
// Created by newnottakenname on 14.12.15.
//

#include <base_catkin/base.h>
#include "BaseCommunicator.h"

/**
 * Start the engines, ready,... set,... go!
 */
int main(int argc, char **argv)
{

    ros::init(argc, argv, "base");

    BaseCommunicator communicator;
    communicator.init();


    return 0;
}

void BaseCommunicator::init() {
    base.init();
    base.spin();

    vel_sub_ = nh_.subscribe("/basecontrol", 50, &Base::velocityCallback, this);
}



BaseCommunicator::Spin()
{
    //ROS_INFO("Spinning %f",curpos);

    ros::Rate r(100);
    while(ros::ok()) {
        base.spin();
        ros::spinOnce();
        r.sleep();
    }
}
