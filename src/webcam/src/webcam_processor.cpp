#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "squarefinder.h"
#include "Square.h"

using namespace cv;
using namespace std;

Point2f computeIntersect(Vec2f line1, Vec2f line2);
vector<Point2f> lineToPointPair(Vec2f line);
bool acceptLinePair(Vec2f line1, Vec2f line2, float minTheta);

void getPositionOfSquares(Square pSquare[50], std::vector<std::vector<Point> > &vector);

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    double re = drand48();
    if(rand() % 100 > 15){
        return;
    }
    //const sensor_msgs::Image::ConstPtr& msg
    //ROS_INFO("I heard: [%d]", msg->data);
    //
    cv::Mat webcam_image= cv_bridge::toCvCopy(msg)->image;
    //cv::Mat webcam_image = cv::imread("/home/newnottakenname/Coding/Tesseron/src/webcam/src/LilStone.jpg", cv::IMREAD_ANYCOLOR);

    if(webcam_image.empty())
    {
        return;
    }
    resize(webcam_image, webcam_image, Size(0, 0), 1, 1);

    vector<vector<Point> > squares;

    //Convert BGR to HSV
    Mat HSVmat;
    cvtColor(webcam_image, HSVmat, COLOR_BGR2HSV);

    //Threshold the HSV image to get only blue colors
    Mat greyMask;
    inRange(HSVmat, Scalar(12,25,30), Scalar(29,120,170), greyMask);
    Mat whiteMask;
    inRange(HSVmat, Scalar(0, 0, 220), Scalar(63, 50, 360), whiteMask);
    Mat blueMask;
    inRange(HSVmat, Scalar(5, 110, 111), Scalar(27, 256, 255), blueMask);
    Mat blackMask;
    inRange(HSVmat, Scalar(0, 0, 0), Scalar(256, 256, 100), blackMask);

    squarefinder finder;
    finder.findSquares(blueMask, squares);
    finder.findSquares(blackMask, squares);
    finder.findSquares(whiteMask, squares);
    finder.findSquares(greyMask, squares);
//    finder.findSquares(invertedPicture, squares);

//    imshow("Windows", greyMask);
    //waitKey(0);


//    imshow("Grey", greyRes);
//    imshow("Blue", blueRes);
//    imshow("White", whiteRes);
//    imshow("Black", blackRes);
//    imshow("Original", webcam_image);

    //imshow("intersect", occludedSquare);
    //waitKey(0);

    Square positions [50];
    getPositionOfSquares(positions, squares);

    finder.drawSquares(webcam_image, squares);

    return;
}

void getPositionOfSquares(Square pSquare[50], std::vector<std::vector<Point> > &vector) {

    double total_width = 0.0;
    double total_height = 0.0;
    int number = 0;

//    std::vector<Point> points;

//    for(int i = 0; i < vector.size(); i++)
//    {
//        std::vector<Point> vic = vector[i];
//        if(vic.size() == 0.0){
//            break;
//        }
//        Point ld = vic[0];
//        Point lu = vic[1];
//        Point ru = vic[1];
//        Point rd = vic[1];
//
//        number++;
//
//        total_height += ((lu.y - ld.y) + (ru.y - rd.y))/2;
//        total_width += ((ru.x - lu.x) + (rd.x - ld.x))/2;
//
//        points.push_back(ld);
//
//    }

    std::vector<std::vector<Point> > singleSquare;

    for(int i = 0; i < vector.size(); i++)
    {
        std::vector<Point> vic = vector[i];
        bool breaked = false;
        for(int j = 0; j < singleSquare.size(); j++){
            std::vector<Point> vic2 = singleSquare[j];
            for(int k = 0; k < 4; k++){
                if(abs(vic[k].x - vic2[k].x) < 30 && abs(vic[k].y - vic2[k].y) < 30 ){
                    int x = abs(vic[k].x - vic2[k].x);
                    int y = abs(vic[k].y - vic2[k].y);
                    breaked = true;
                }
            }
            if(breaked){
                break;
            }
        }
        if(breaked)
        {
            continue;
        }
        Point ld = vic[0];
        Point lu = vic[1];
        Point ru = vic[2];
        Point rd = vic[3];

        double west =  sqrt(pow(lu.y - ld.y,2) + pow(lu.x - ld.x,2));
        double east =  sqrt(pow(ru.y - rd.y,2) + pow(ru.x - rd.x,2));
        double north = sqrt(pow(lu.y - ru.y,2) + pow(lu.x - ru.x,2));
        double south = sqrt(pow(ld.y - rd.y,2) + pow(ld.x - rd.x,2));

        double average = (west+ east + north+ south)/4;

        double temp = abs(east - west + north - south);
        if(temp > 20)
        {
            continue;
        }

        singleSquare.push_back(vic);

    }

    vector = singleSquare;
    //bool pause = true;


}

bool acceptLinePair(Vec2f line1, Vec2f line2, float minTheta)
{
    float theta1 = line1[1], theta2 = line2[1];

    if(theta1 < minTheta)
    {
        theta1 += CV_PI; // dealing with 0 and 180 ambiguities...
    }

    if(theta2 < minTheta)
    {
        theta2 += CV_PI; // dealing with 0 and 180 ambiguities...
    }

    return abs(theta1 - theta2) > minTheta;
}

// the long nasty wikipedia line-intersection equation...bleh...
Point2f computeIntersect(Vec2f line1, Vec2f line2)
{
    vector<Point2f> p1 = lineToPointPair(line1);
    vector<Point2f> p2 = lineToPointPair(line2);

    float denom = (p1[0].x - p1[1].x)*(p2[0].y - p2[1].y) - (p1[0].y - p1[1].y)*(p2[0].x - p2[1].x);
    Point2f intersect(((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].x - p2[1].x) -
                       (p1[0].x - p1[1].x)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom,
                      ((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].y - p2[1].y) -
                       (p1[0].y - p1[1].y)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom);

    return intersect;
}

vector<Point2f> lineToPointPair(Vec2f line)
{
    vector<Point2f> points;

    float r = line[0], t = line[1];
    double cos_t = cos(t), sin_t = sin(t);
    double x0 = r*cos_t, y0 = r*sin_t;
    double alpha = 1000;

    points.push_back(Point2f(x0 + alpha*(-sin_t), y0 + alpha*cos_t));
    points.push_back(Point2f(x0 - alpha*(-sin_t), y0 - alpha*cos_t));

    return points;
}

int main(int argc, char **argv)
{
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
  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 1000, chatterCallback);
    //chatterCallback();

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
