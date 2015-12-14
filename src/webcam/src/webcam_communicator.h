//
// Created by newnottakenname on 10.12.15.
//

#ifndef WEBCAM_WEBCAM_COMMUNICATOR_H
#define WEBCAM_WEBCAM_COMMUNICATOR_H

#include "webcam_communicator.h"
#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "squarefinder.h"
#include "Tile.h"
#include "std_msgs/String.h"

using namespace cv;
using namespace std;

ros::Publisher ros_pub;
bool IsWorkingOnImage;





#endif //WEBCAM_WEBCAM_COMMUNICATOR_H
