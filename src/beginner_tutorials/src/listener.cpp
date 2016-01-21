#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace cv;
using namespace std;

Point2f computeIntersect(Vec2f line1, Vec2f line2);
vector<Point2f> lineToPointPair(Vec2f line);
bool acceptLinePair(Vec2f line1, Vec2f line2, float minTheta);

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    //ROS_INFO("I heard: [%d]", msg->data);
    cv::Mat occludedSquare = cv_bridge::toCvCopy(msg)->image;
    //cv::Mat im = cv::imread("/home/newnottakenname/Coding/Tesseron/src/webcam/src/Mosaic.jpg", cv::IMREAD_GRAYSCALE);

    if(occludedSquare.empty())
    {
        return;
    }

    resize(occludedSquare, occludedSquare, Size(0, 0), 0.5, 0.5);

    Mat occludedSquare8u;
    cvtColor(occludedSquare, occludedSquare8u, CV_BGRA2GRAY);

    Mat thresh;
    threshold(occludedSquare8u, thresh, 200.0, 255.0, THRESH_BINARY);

    GaussianBlur(thresh, thresh, Size(7, 7), 2.0, 2.0);

    Mat edges;
    Canny(thresh, edges, 66.0, 133.0, 3);

    vector<Vec2f> lines;
    HoughLines( edges, lines, 1, CV_PI/180, 50, 0, 0 );

    cout << "Detected " << lines.size() << " lines." << endl;

    // compute the intersection from the lines detected...
    vector<Point2f> intersections;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        for(size_t j = 0; j < lines.size(); j++)
        {
            Vec2f line1 = lines[i];
            Vec2f line2 = lines[j];
            if(acceptLinePair(line1, line2, CV_PI / 32))
            {
                Point2f intersection = computeIntersect(line1, line2);
                intersections.push_back(intersection);
            }
        }

    }

    if(intersections.size() > 0)
    {
        vector<Point2f>::iterator i;
        for(i = intersections.begin(); i != intersections.end(); ++i)
        {
            cout << "Intersection is " << i->x << ", " << i->y << endl;
            circle(occludedSquare, *i, 1, Scalar(0, 255, 0), 3);
        }
    }

    imshow("intersect", occludedSquare);
    waitKey(30);

    return;
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

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
