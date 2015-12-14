//
// Created by newnottakenname on 14.12.15.
//

#ifndef WEBCAM_WEBCAM_PROCESSOR_H
#define WEBCAM_WEBCAM_PROCESSOR_H

#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "squarefinder.h"
#include "std_msgs/String.h"
#include "Tile.h"

using namespace cv;
using namespace std;


void getPositionOfSquares(std::vector<Tile> pSquare, std::vector<std::vector<Point> > &vector);
void ProcessImage(cv::Mat webcam_image, std::vector<Tile> tiles);



#endif //WEBCAM_WEBCAM_PROCESSOR_H
