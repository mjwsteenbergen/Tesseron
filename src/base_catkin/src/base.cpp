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

	// Publish status
	// code here!

	// Load motor configuration
	CXMLConfiguration motor_config_xml;
	ROS_ASSERT(motor_config_xml.loadFile(motor_config_name));

	CDxlConfig motor_config_left;
	motor_config_left.readConfig(motor_config_xml.root().section("left"));
	left_motor_ = new C3mxlROS(motor_port_name.c_str());
	left_motor_->setConfig(&motor_config_left);

	// Initialize left motor
	ros::Rate init_rate(1);
	while (ros::ok() && left_motor_->init() != DXL_SUCCESS) {
		ROS_WARN_ONCE("Couldn't initialize left wheel motor, will continue trying every second");
		init_rate.sleep();
	}

	CDxlConfig motor_config_right;
	motor_config_right.readConfig(motor_config_xml.root().section("right"));
	right_motor_ = new C3mxlROS(motor_port_name.c_str());
	right_motor_->setConfig(&motor_config_right);

	// Initialize right motor
	while (ros::ok() && right_motor_->init() != DXL_SUCCESS) {
		ROS_WARN_ONCE("Couldn't initialize right wheel motor, will continue trying every second");
		init_rate.sleep();
	}

	DXL_SAFE_CALL(left_motor_->set3MxlMode(motor_config_left.m3mxlMode));
	DXL_SAFE_CALL(right_motor_->set3MxlMode(motor_config_right.m3mxlMode));

	mode_pos_ = false;

	ROS_INFO("Base initialized");
}

void Base::spin() {
	ROS_INFO("Spinning");

	ros::Rate r(100);

	while(ros::ok()) {
		ros::spinOnce();
		publishStatus();
		r.sleep();
	}
}

/**
 * Callback that handles velocities
 */
void Base::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg) {
	// Base is nonholonomic, warn if sent a command we can't execute
	if (msg->linear.y || msg->linear.z || msg->angular.x || msg->angular.y) {
		ROS_WARN("I'm afraid I can't do that, Dave.");
		return;
	}

	if(mode_pos_) {
		left_motor_->set3MxlMode(SPEED_MODE);
		left_motor_->get3MxlMode();

		right_motor_->set3MxlMode(SPEED_MODE);
		right_motor_->get3MxlMode();

		mode_pos_ = false;
	}

	//	more stuff here
}

/**
 * Publish the status of the base motors
 */
void Base::publishStatus() {
	base_catkin::BaseStatus msg;
	
	//	more stuff here

	status_pub_.publish(msg);
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



