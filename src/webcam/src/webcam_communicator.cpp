//
// Created by newnottakenname on 10.12.15.
//

#include "webcam_communicator.h"
#include "webcam_processor.hpp"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const sensor_msgs::Image::ConstPtr &msg) {

    IsWorkingOnImage = true;
//    double re = drand48();
//    if (rand() % 100 > 15) {
//        return;
//    }
    //const sensor_msgs::Image::ConstPtr& msg
    //ROS_INFO("I heard: [%d]", msg->data);
    //
    cv::Mat webcam_image;
    webcam_image = cv_bridge::toCvCopy(msg)->image;

    std::vector<Tile> tiles;
    ProcessImage(webcam_image, tiles);

    IsWorkingOnImage = false;
    //cv::Mat webcam_image = cv::imread("/home/newnottakenname/Coding/Tesseron/src/webcam/src/LilStone.jpg", cv::IMREAD_ANYCOLOR);


}

int main(int argc, char **argv) {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "listener");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called chatterCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     */
    ros_pub = n.advertise<std_msgs::String>("/Tesseron/", 1000);
    ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 1, chatterCallback);

    IsWorkingOnImage = false;

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}
