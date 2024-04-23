// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim
//
// Modified by Claude Sammut for COMP3431
// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <memory>

using namespace std::chrono_literals;

WallFollower::WallFollower()
: Node("wall_follower_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	scan_data_[0] = 0.0;
	scan_data_[1] = 0.0;
	scan_data_[2] = 0.0;

	robot_pose_ = 0.0;
	prev_robot_pose_ = 0.0;

	start_x = 0.0;
	start_y = 0.0;
	
	x = 0.0;
	y = 0.0;

	mov_count = 0;

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

	// Initialise subscribers
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", \
		rclcpp::SensorDataQoS(), \
		std::bind(
			&WallFollower::scan_callback, \
			this, \
			std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

	/************************************************************
	** Initialise ROS timers
	************************************************************/
	update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this));

	this->declare_parameter("robot_should_stop", false);

	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

WallFollower::~WallFollower()
{
	RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	
	double current_x = msg->pose.pose.position.x;
	double current_y = msg->pose.pose.position.y;

	
	if(start_x == 0) {
		start_x = current_x;
		start_y = current_y;	
	}
	x = current_x;
	y = current_y;
	robot_pose_ = yaw;
}

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle[3] = {0, 330, 300}; // was [RIGHT] = 310

	for (int num = 0; num < 3; num++)
	{
		if (std::isinf(msg->ranges.at(scan_angle[num])))
		{
			scan_data_[num] = msg->range_max;
		}
		else
		{
			scan_data_[num] = msg->ranges.at(scan_angle[num]);
		}
	}
}

void WallFollower::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void WallFollower::update_callback()
{
	
	// Before sending a command
	bool should_stop;
	this->get_parameter("robot_should_stop", should_stop);
	if (should_stop) {
	    update_cmd_vel(0.0, 0.0);
	    return;
	}
	// End parameter

	static uint8_t turtlebot3_state_num = 1;

	double check_forward_dist = 0.4;
	// if find a big right space tracking right
	double check_big_side_dist = 0.7;
	// if too close to right it should turn left
	double check_small_side_dist = 0.45;
	mov_count += 1;
	double finish_distance = 0.2;
	if((fabs(start_x - x) <= finish_distance) && (fabs(start_y - y) <= finish_distance)&& mov_count > 2000) {
		update_cmd_vel(0,0);
		std::cout << "DONE!" << std::endl;
		turtlebot3_state_num = TB3_DONE;
			
	}
		//std::cout << "Current state: " << static_cast<int>(turtlebot3_state_num) << std::endl;
		//std::cout << "The forward_dist is: " << scan_data_[CENTER] << std::endl;
		//std::cout << "The right dist is: " << scan_data_[RIGHT] << std::endl;
		//std::cout << "The right center dist is: " << scan_data_[RIGHT_CENTER] << std::endl;
		std::cout << "The x is: " << x << " vs start x: " << start_x << std::endl;
		std::cout << "The y is: " << y << " vs start y: " << start_y << std::endl;

	switch (turtlebot3_state_num)
	{

		case TB3_DRIVE_FORWARD:
			if (scan_data_[CENTER] == 0 || scan_data_[RIGHT_CENTER] == 0 || scan_data_[RIGHT] == 0) {
				turtlebot3_state_num = TB3_DRIVE_FORWARD;
				break;
			}
			//too close to the front, we keep turning until we have enough front space
			if (scan_data_[CENTER] < check_forward_dist || scan_data_[RIGHT_CENTER] < 0.4) {
				turtlebot3_state_num = TB3_LEFT_TURN;

			}
			// if we still have some right space, we right follow the wall
			else if (scan_data_[RIGHT] > check_big_side_dist) {
				turtlebot3_state_num = TB3_FOLLOW_RIGHT;			
			}
			// else keep moving forward
			else {
				update_cmd_vel(LINEAR_VELOCITY, 0.0);
			}
			break;
		case TB3_FOLLOW_RIGHT:
			if (scan_data_[CENTER] < check_forward_dist) {
				turtlebot3_state_num = TB3_LEFT_TURN;

			}
			//too close to the right, we then need to turn left until there is space
			else if (scan_data_[RIGHT] < check_small_side_dist) {
				turtlebot3_state_num = TB3_LEFT_TURN;			
			}
			// if center is too close we need to somehow adjust it in the big right arc.
			//else if(scan_data_[RIGHT_CENTER] < 0.3){
			//	update_cmd_vel(0.1, 0 * ANGULAR_VELOCITY);
			//	turtlebot3_state_num = TB3_ADJUST_DIST;	
			//}
			else {
				update_cmd_vel(0.13, -0.45 * ANGULAR_VELOCITY);			
			}
			break;

		case TB3_DONE: 
 			
			
			break;

		case TB3_LEFT_TURN:
			if (scan_data_[CENTER] > check_forward_dist && scan_data_[RIGHT_CENTER] > 0.4) {
				turtlebot3_state_num = TB3_DRIVE_FORWARD;

			}
			else
			{
				update_cmd_vel(0.0, 0.6*ANGULAR_VELOCITY);
			}
			break;

		default:
			turtlebot3_state_num = 1;
			break;
	}
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();

	return 0;
}
