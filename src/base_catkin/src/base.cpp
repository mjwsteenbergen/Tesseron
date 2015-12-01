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

#define CLIP(x, l, u) ((x)<(l))?(l):((x)>(u))?(u):(x)


void Base::init() {
	ROS_INFO("Initializing base");

	// Read parameters
	std::string motor_port_name, motor_config_name;

	ROS_ASSERT(nh_.getParam("motor_port", motor_port_name));
	ROS_ASSERT(nh_.getParam("motor_config", motor_config_name));
	ROS_ASSERT(nh_.getParam("wheel_diameter", wheel_diameter_));
	ROS_ASSERT(nh_.getParam("wheel_base", wheel_base_));
	


	// Subscriptions to command topic
	// code here!
	ros::Subscriber sub = nh_.subscribe<geometry_msgs::Twist>("basecontrol", 50, &Base::velocityCallback, this);
	// Publish status
	// code here!

	// Load motor configuration
	CXMLConfiguration motor_config_xml;
	ROS_ASSERT(motor_config_xml.loadFile(motor_config_name));

	CDxlConfig motor_config_left;
	motor_config_left.readConfig(motor_config_xml.root().section("left"));
	left_motor_ = new C3mxlROS(motor_port_name.c_str());
	left_motor_->setConfig(&motor_config_left);

	//ROS_INFO("Initializing base10");

	// Initialize left motor
	ros::Rate init_rate(1);
	while (ros::ok() && left_motor_->init() != DXL_SUCCESS) {
		ROS_WARN_ONCE("Couldn't initialize left wheel motor, will continue trying every second");
		init_rate.sleep();
	}
	//ROS_INFO("Initializing base11");
	CDxlConfig motor_config_right;
	motor_config_right.readConfig(motor_config_xml.root().section("right"));
	right_motor_ = new C3mxlROS(motor_port_name.c_str());
	right_motor_->setConfig(&motor_config_right);

	//ROS_INFO("Initializing base2");

	// Initialize right motor
	while (ros::ok() && right_motor_->init() != DXL_SUCCESS) {
		ROS_WARN_ONCE("Couldn't initialize right wheel motor, will continue trying every second");
		init_rate.sleep();
	}

	DXL_SAFE_CALL(left_motor_->set3MxlMode(motor_config_left.m3mxlMode));
	DXL_SAFE_CALL(right_motor_->set3MxlMode(motor_config_right.m3mxlMode));

	mode_pos_ = false;

	ROS_INFO("Base initialized");
	//left_motor_->set3MxlMode(SPEED_MODE);
}


void Base::spin() {
	double curpos;
	curpos = left_motor_->getPos();
	ROS_INFO("Spinning %f",curpos);

	ros::Rate r(100);
	drive();
	while(ros::ok()) {
		left_motor_->getState();
		curpos = left_motor_->presentPos();
		ROS_INFO("Position %f",curpos);
		ros::spinOnce();
		//publishStatus();
		r.sleep();
	}
}

void Base::drive() {
	ROS_INFO("Driving");
}

/**
 * Callback that handles velocities
 */

void Base::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg) {

		ROS_INFO("Message recieved");		
		left_motor_->set3MxlMode(SPEED_MODE);
		left_motor_->get3MxlMode();
		left_motor_->setLinearSpeed(msg->linear.y + msg->angular.z); 
		right_motor_->set3MxlMode(SPEED_MODE);
		right_motor_->get3MxlMode();
		right_motor_->setLinearSpeed(msg->linear.y - msg->angular.z); 

}



/**
 * Publish the status of the base motors
 */
void Base::publishStatus() {
	base_catkin::BaseStatus msg;
	ros::Publisher ztatuz_pub_ = nh_.advertise<base_catkin::BaseStatus>("/motor_comm/sendto", 1000);
	
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
	
	ros::init(argc, argv, "base");
	Base base;
	base.spin();

	return 0;
}



