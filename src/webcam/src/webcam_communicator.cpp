//
// Created by newnottakenname on 10.12.15.
//

#include "webcam_communicator.h"
#include "webcam_processor.hpp"
#include "webcam/Tile_Message.h"
#include "webcam/Webcam_Message.h"
#include <sensor_msgs/image_encodings.h>
#include "cv_bridge/cv_bridge.h"
#include <ros/ros.h>

const double TILE_HEIGHT = 10.0;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const sensor_msgs::Image::ConstPtr &msg) {

//    cv::Mat webcam_image = cv::imread("/home/newnottakenname/Coding/Tesseron/src/webcam/src/LilStone.jpg", cv::IMREAD_ANYCOLOR);

    cv::Mat webcam_image;



//    sensor_msgs::Image current_rgb_msg;
//    current_rgb_msg = *msg;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
//sensor_msgs::image_encodings::BGR8
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    webcam_image = cv_ptr->image;

    std::vector<Tile> tiles;
    ProcessImage(webcam_image, tiles);

    if(tiles.size() < 2)
    {
        return;
    }


    Tile left_before;
    Tile left = tiles[0];
    Tile up = tiles[0];

    for (int i = 0; i < tiles.size(); ++i) {
        Tile item = tiles[i];
        if(item.GetX() > left.GetX() && item.GetY() > left.GetY()){
            left = item;
        }
    }

    int smallest_dif = 900000;
    for (int i = 0; i < tiles.size(); ++i) {
        Tile item = tiles[i];
        double difT = item.GetX() - left.GetX();
        if(difT < 0){
            difT= difT * -1;
        }
        if(difT < smallest_dif && item.GetY() < up.GetY() && item.GetY() > TILE_HEIGHT + left.GetY()){
            left = item;
        }
    }

    webcam::Tile_Message left_m;
    left_m.x = left.GetX();
    left_m.y = left.GetY();

    webcam::Tile_Message up_m;
    up_m.x = up.GetX();
    up_m.y = up.GetY();

    webcam::Webcam_Message message;
    message.left = left_m;
    message.up = up_m;


    webcam_pub.publish(message);

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
    ros::init(argc, argv, "webcam");


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
//    webcam_pub = n.advertise("/Tesseron/webcam", 1000);

    ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 1, chatterCallback);

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}
