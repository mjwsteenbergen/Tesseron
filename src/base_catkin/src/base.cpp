/*
 * base.cpp
 *
 * Created on: 	Nov 17, 2013
 *     Author:	Floris Gaisser
 *
 * Versions:
 * 1.0		Initial version
 *
 */
#include <threemxl/platform/io/configuration/XMLConfiguration.h>
#include <threemxl/dxlassert.h>
#include <base_catkin/base.h>
#include <sensor_msgs/Joy.h>
#include "base_catkin/Wheel.h"
#include <std_msgs/Bool.h>

#define CLIP(x, l, u) ((x)<(l))?(l):((x)>(u))?(u):(x)

ros::Publisher ztatuz_pub_;

void Base::init() {
	ROS_INFO("Initializing base");


	initBase();


	// Subscriptions to command topic
	// code here!
	vel_sub_ = nh_.subscribe("/basecontrol", 50, &Base::velocityCallback, this);
	ztatuz_pub_ = nh_.advertise<base_catkin::BaseStatus>("/base/status", 1000);

//	statusPublisher = nh_.advertise<base_catkin::StatusMessage>("Tesseron/init", 10);

//	 = nh_.subscribe("/test",1,&Base::testcb,this);

	// Publish status
	// code here!



	wheelClient = nh_.advertiseService("/base/Wheels", &Base::drive, this);
	initPublisher = nh_.advertise<std_msgs::Bool>("/Tesseron/init", 10);

	cur_pos = 0;
	drive(0);

	sendInitMessage();

	ROS_INFO("Base initialized");
	//right_motor_->set3MxlMode(SPEED_MODE);

	initService();
}

void Base::initBase()
{
	// Read parameters
	std::string motor_port_name, motor_config_name;

	ROS_ASSERT(nh_.getParam("motor_port", motor_port_name));
	ROS_ASSERT(nh_.getParam("motor_config", motor_config_name));
	ROS_ASSERT(nh_.getParam("wheel_diameter", wheel_diameter_));
	ROS_ASSERT(nh_.getParam("wheel_base", wheel_base_));

	// Load motor configuration
	CXMLConfiguration motor_config_xml;
	ROS_ASSERT(motor_config_xml.loadFile(motor_config_name));

	CDxlConfig motor_config_left;
	motor_config_left.readConfig(motor_config_xml.root().section("left"));
	right_motor_ = new C3mxlROS(motor_port_name.c_str());
	right_motor_->setConfig(&motor_config_left);


	//ROS_INFO("Initializing base10");

	// Initialize left motor
	ros::Rate init_rate(1);
	while (ros::ok() && right_motor_->init() != DXL_SUCCESS) {
		ROS_WARN_ONCE("Couldn't initialize left wheel motor, will continue trying every second");
		init_rate.sleep();
	}
	//ROS_INFO("Initializing base11");
	CDxlConfig motor_config_right;
	motor_config_right.readConfig(motor_config_xml.root().section("right"));
	left_motor_ = new C3mxlROS(motor_port_name.c_str());
	left_motor_->setConfig(&motor_config_right);
	//ROS_INFO("Initializing base2");

	// Initialize right motor
	while (ros::ok() && left_motor_->init() != DXL_SUCCESS) {
		ROS_WARN_ONCE("Couldn't initialize right wheel motor, will continue trying every second");
		init_rate.sleep();
	}

	DXL_SAFE_CALL(right_motor_->set3MxlMode(motor_config_left.m3mxlMode));
	DXL_SAFE_CALL(left_motor_->set3MxlMode(motor_config_right.m3mxlMode));


	left_motor_->set3MxlMode(SPEED_MODE);
	right_motor_->set3MxlMode(SPEED_MODE);

	mode_pos_ = false;

	ROS_INFO("The base has been initialised");
}

void Base::sendInitMessage()
{
    ros::Rate rate(1);
    ros::spinOnce();
    rate.sleep();
	std_msgs::Bool msg;
	msg.data = (unsigned char) true;
	initPublisher.publish(msg);
	ros::spinOnce();
}

void Base::initService() {
    //ros::ServiceServer service = nh_.advertiseService("add_two_ints", drive);
}

bool Base::drive(base_catkin::Wheel::Request &req, base_catkin::Wheel::Response &res){
	std::string error = drive(req.distance);
	if(error != "")
	{
		res.succeeded =false;
		res.error = error;
	}
	else
	{
		res.succeeded = true;
		res.error = "";
	}
}

std::string Base::drive(float distance)
{
	int state = right_motor_->getState();
	int state2 = left_motor_->getState();

	if(right_motor_->present3MxlMode() != POSITION_MODE || left_motor_->present3MxlMode() != POSITION_MODE)
	{
		right_motor_ ->set3MxlMode(POSITION_MODE);
		left_motor_->set3MxlMode(POSITION_MODE);
	}


    double startPosRadL = right_motor_->presentLinearPos();
    double startPosRadR = left_motor_->presentLinearPos();

//    double speed = (distance / 0.072) * 2 * M_PI;
    //
    right_motor_ ->setLinearPos((cur_pos + distance) * 2 * 1.1, 0.1*1.025, 0.05*1.018, false);
    left_motor_->setLinearPos((cur_pos + distance) * 2, 0.1, 0.05, false);

	ros::Rate sleep(10);
	for (int i = 0; i < 200; ++i) {
//	}(right_motor_->presentSpeed() > 0 || left_motor_->presentSpeed() > 0)
//	{
		right_motor_->getState();
		left_motor_->getState();
//	    std::to_string(0.1);
//		std::string s = std::to_string(right_motor_->presentPos()).append(" - ").append(std::to_string(left_motor_->presentPos()));
//		std::string s2 = std::to_string(right_motor_->presentSpeed()).append(" - ").append(std::to_string(left_motor_->presentSpeed()));
//		ROS_INFO(s.c_str());
//		ROS_WARN(s2.c_str());
		sleep.sleep();
//
	}

	cur_pos += distance;

	return "";
}


void Base::testcb(const std_msgs::Empty::ConstPtr &msg)
{
	ROS_INFO("here");
}

void Base::spin() {
	ros::Rate r(100);
	//drive();
	while(ros::ok()) {
		right_motor_->getLinearPos();
		left_motor_->getLinearPos();
		//ROS_INFO("Position %f",curpos);
		ros::spinOnce();
		//publishStatus();
		r.sleep();
	}
}

/**
 * Callback that handles velocities
 */

void Base::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg) {
	setSpeed(msg->linear.x, msg->linear.y);
}

void Base::setSpeed(double speedLeft, double speedRight)
{
	if(right_motor_->present3MxlMode() != SPEED_MODE)
	{
		right_motor_->set3MxlMode(SPEED_MODE);
	}
	if(left_motor_->present3MxlMode() != SPEED_MODE)
	{
		left_motor_->set3MxlMode(SPEED_MODE);
	}


	ROS_INFO("Setting different speeds");
	right_motor_->setSpeed(speedLeft);
	left_motor_->setSpeed(speedRight);
}





/**
 * Publish the status of the base motors
 */
void Base::publishStatus() {
	base_catkin::BaseStatus msg;

	//	more stuff here
	base_catkin::MotorStatus ll;
	base_catkin::MotorStatus rr;
	msg.left = ll;
	msg.right = rr;
	msg.left.speed = 1;
	msg.left.position = 1;
	msg.left.torque = 0.01;
	msg.right.speed = 1;
	msg.right.position = 1;
	msg.right.torque = 0.01;

	ztatuz_pub_.publish(msg);
}

/**
 * Start the engines, ready,... set,... go!
 */
int main(int argc, char **argv)
{
	ROS_INFO("Starting node...");

	ros::init(argc, argv, "base");
	Base base;

//	ROS_INFO("Powering engines");

//	base.setSpeed(3.0,3.0);

//	ros::Rate sleep(1.0);
//	sleep.sleep();

//	base.setSpeed(0.0, 0);

//	ROS_INFO("Moving engines");

//	float distance = -0.5;
//	base.drive(-0.5);
//	for (int i = 0; i < 10; ++i) {
//		distance += 0.025;
//		std::string info = "At: ";
//		ROS_INFO(info.append(std::to_string(i)).c_str());
//		base.drive(0.025);
//	}
//	base.drive(0.5);

//	ROS_INFO("Moving second pair of engines");

//	base.drive(-0.5);
	base.spin();

	ROS_INFO("I'm OUTTA HERE GOODBYE AND THANKS FOR THE FISH");
	ros::spin();
	return 0;
}



