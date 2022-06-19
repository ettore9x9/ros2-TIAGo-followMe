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

#include "turtlebot3_drive.hpp"

using namespace std::chrono_literals;

Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

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
      &Turtlebot3Drive::scan_callback, \
      this, \
      std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&Turtlebot3Drive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialised");
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void Turtlebot3Drive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;
}

void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
 
  std::vector<float> laser_ranges;
  laser_ranges = msg->ranges;
 
  range_min = msg->range_max; 
  range_max = msg->range_min;

  float crashed_min = 0.25;
  side_min = 3.5;

  front_side_min = side_min;
  front_left_side_min = side_min;
  front_right_side_min = side_min;
  left_side_min = side_min;
  right_side_min = side_min;
  behind_side_min = side_min;
  
  for (int a = 0; a < 360; a++) {
    if (a >= 30 && a <= 330){
      continue;
    }
    if (laser_ranges[a] < front_side_min && laser_ranges[a] > 0) {
          front_side_min = laser_ranges[a];
      }
  }

  for (int a = 30; a < 60; a++) {
    if (laser_ranges[a] < front_left_side_min && laser_ranges[a] > 0) {
          front_left_side_min = laser_ranges[a];
      }
  }

  for (int a = 300; a < 330; a++) {
    if (laser_ranges[a] < front_right_side_min && laser_ranges[a] > 0) {
          front_right_side_min = laser_ranges[a];
      }
  }

  for (int a = 60; a < 90; a++) {
    if (laser_ranges[a] < left_side_min && laser_ranges[a] > 0) {
          left_side_min = laser_ranges[a];
      }
  }

  for (int a = 270; a < 300; a++) {
    if (laser_ranges[a] < right_side_min && laser_ranges[a] > 0) {
          right_side_min = laser_ranges[a];
      }
  }

  for (int a = 90; a < 270; a++) {
    if (laser_ranges[a] < behind_side_min && laser_ranges[a] > 0) {
          behind_side_min = laser_ranges[a];
      }
  }

  if (front_side_min < crashed_min || front_right_side_min < crashed_min || right_side_min < crashed_min) {
    if (behind_side_min > 1.0){
      crashed = true;
    }
  }
  else {
    crashed = false;
  }

}

void Turtlebot3Drive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void Turtlebot3Drive::update_callback()
{
  
  double distance_min = 0.15;
  double distance_test = 0.4;

  if (front_side_min > distance_min){
    if (!crashed){
      if (front_side_min > distance_test && front_left_side_min > distance_test && front_right_side_min > distance_test){
        tb3_find_wall();
      }
      else if (front_side_min < distance_test && front_left_side_min > distance_test && front_right_side_min > distance_test){
        tb3_turn_left(); 
      }
      else if (front_side_min > distance_test && front_left_side_min > distance_test && front_right_side_min < distance_test){
        tb3_turn_left(); 
      }
      else if (front_side_min > distance_test && front_left_side_min < distance_test && front_right_side_min > distance_test){
        tb3_turn_right(); 
      }
      else if (front_side_min < distance_test && front_left_side_min > distance_test && front_right_side_min < distance_test){
        tb3_turn_left(); 
      }
      else if (front_side_min < distance_test && front_left_side_min < distance_test && front_right_side_min > distance_test){
        tb3_turn_right(); 
      }  
      else if (front_side_min < distance_test && front_left_side_min < distance_test && front_right_side_min < distance_test){
        tb3_turn_left(); 
      }     
      else if (front_side_min > distance_test && front_left_side_min < distance_test && front_right_side_min < distance_test){
        tb3_turn_left(); 
      }          
    }
    else {
      tb3_move_backward();
    }
  }
  else {
    tb3_full_stop();
  }

  RCLCPP_INFO(this->get_logger(), "scan data at front: %lf ", front_side_min);
  printf("[STATE INFO]:  : [%s]\n",robot_pos_state_);
}

void Turtlebot3Drive::tb3_move_backward()
{
  update_cmd_vel(-1.0 * LINEAR_VELOCITY, 0.0); 
  robot_pos_state_ = "Moving Backward";
}

void Turtlebot3Drive::tb3_full_stop()
{
  update_cmd_vel(0.0, 0.0); 
  robot_pos_state_ = "Full Stop";
}

void Turtlebot3Drive::tb3_find_wall()
{
  update_cmd_vel(LINEAR_VELOCITY, 0.0); 
  robot_pos_state_ = "Finding The Wall";
}

void Turtlebot3Drive::tb3_turn_left()
{
  update_cmd_vel(0.0, ANGULAR_VELOCITY);
  robot_pos_state_ = "Turning Left";
}

void Turtlebot3Drive::tb3_turn_right()
{
  update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY);
  robot_pos_state_ = "Turning Right";
}

void Turtlebot3Drive::tb3_follow_the_wall()
{
  update_cmd_vel(LINEAR_VELOCITY, 0.0); 
  robot_pos_state_ = "Following The Wall";
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>());
  rclcpp::shutdown();

  return 0;
}
