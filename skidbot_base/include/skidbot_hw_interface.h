/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the RRBot
           For a more detailed simulation example, see sim_hw_interface.h
*/

#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h> // esto lo saque de otro lado

#include <boost/assign/list_of.hpp>

// NaN
#include <limits> // esto lo saque de otro lado
// ostringstream
#include <sstream>

const unsigned int NUM_JOINTS = 4;
const float _PI = 3.1415926535897931;

/// \brief Hardware interface for a robot
class MyRobotHWInterface : public hardware_interface::RobotHW
{
public:
  MyRobotHWInterface();

  /*
   *
   */
  void write() {
    double diff_ang_speed_front_left = cmd[0] * 0.5 * _wheel_diameter;
    double diff_ang_speed_front_right = cmd[1] * 0.5 * _wheel_diameter;
    double diff_ang_speed_rear_left = cmd[2] * 0.5 * _wheel_diameter;
    double diff_ang_speed_rear_right = cmd[3] * 0.5 * _wheel_diameter;
      // limitDifferentialSpeed(diff_ang_speed_left, diff_ang_speed_right);
  	// Publish results
  	std_msgs::Float32 front_left_wheel_vel_msg;
  	std_msgs::Float32 front_right_wheel_vel_msg;
    std_msgs::Float32 rear_left_wheel_vel_msg;
  	std_msgs::Float32 rear_right_wheel_vel_msg;

  	front_left_wheel_vel_msg.data = diff_ang_speed_front_left;
    front_right_wheel_vel_msg.data = diff_ang_speed_front_right;
    rear_left_wheel_vel_msg.data = diff_ang_speed_rear_left;
    rear_right_wheel_vel_msg.data = diff_ang_speed_rear_right;

  	front_left_wheel_vel_pub_.publish(front_left_wheel_vel_msg);
  	front_right_wheel_vel_pub_.publish(front_right_wheel_vel_msg);
    rear_left_wheel_vel_pub_.publish(rear_left_wheel_vel_msg);
    rear_right_wheel_vel_pub_.publish(rear_right_wheel_vel_msg);
  }

  /**
   * Reading encoder values and setting position and velocity of enconders
   */
  void read(const ros::Duration &period) {
    double ang_distance_front_left = _wheel_angle[0] - pos[0];
    double ang_distance_front_right = _wheel_angle[1] - pos[1];
    double ang_distance_rear_left = _wheel_angle[2] - pos[2];
    double ang_distance_rear_right = _wheel_angle[3] - pos[3];

    pos[0] += ang_distance_front_left;
    pos[1] += ang_distance_front_right;
    pos[2] += ang_distance_rear_left;
    pos[3] += ang_distance_rear_right;

    vel[0] = ang_distance_front_left / period.toSec();
    vel[1] = ang_distance_front_right / period.toSec();
    vel[2] = ang_distance_rear_left / period.toSec();
    vel[3] = ang_distance_rear_right / period.toSec();



    // std::ostringstream os;
    // for (unsigned int i = 0; i < NUM_JOINTS - 1; ++i)
    // {
    //   os << cmd[i] << ", ";
    //   pos[i] = cmd[i];
    // }
    // os << cmd[NUM_JOINTS - 1];
    //
    // ROS_INFO_STREAM("Commands for joints: " << os.str());

  }

  ros::Time get_time() {
    prev_update_time = curr_update_time;
    curr_update_time = ros::Time::now();
    return curr_update_time;
  }

  ros::Duration get_period() {
    return curr_update_time - prev_update_time;
  }

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[NUM_JOINTS];
  double pos[NUM_JOINTS];
  double vel[NUM_JOINTS];
  double eff[NUM_JOINTS];

  bool running_;
  double _wheel_diameter;
  double _max_speed;
  double _wheel_angle[NUM_JOINTS];

  ros::Time curr_update_time, prev_update_time;

  ros::Subscriber front_left_wheel_angle_sub_;
  ros::Subscriber front_right_wheel_angle_sub_;
  ros::Subscriber rear_left_wheel_angle_sub_;
  ros::Subscriber rear_right_wheel_angle_sub_;

  ros::Publisher front_left_wheel_vel_pub_;
  ros::Publisher front_right_wheel_vel_pub_;
  ros::Publisher rear_left_wheel_vel_pub_;
  ros::Publisher rear_right_wheel_vel_pub_;

  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;

  bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = false;
    return true;
  }

  void frontLeftWheelAngleCallback(const std_msgs::Int32& msg) {
    _wheel_angle[0] = (float)msg.data * (2 * _PI / 3270.0);
  }

  void frontRightWheelAngleCallback(const std_msgs::Int32& msg) {
    _wheel_angle[1] = (float)msg.data * (2 * _PI / 3270.0);
  }

  void rearLeftWheelAngleCallback(const std_msgs::Int32& msg) {
    _wheel_angle[2] = (float)msg.data * (2 * _PI / 3270.0);
  }

  void rearRightWheelAngleCallback(const std_msgs::Int32& msg) {
    _wheel_angle[3] = (float)msg.data * (2 * _PI / 3270.0);
  }

  // void rightWheelAngleCallback(const std_msgs::Float32& msg) {
  //   // _wheel_angle[1] = msg.data * ((float) 2*_PI / 16383);
  //    _wheel_angle[1] = msg.data
  // }

  // void limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  // {
	// double speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));
	// if (speed > _max_speed) {
	// 	diff_speed_left *= _max_speed / speed;
	// 	diff_speed_right *= _max_speed / speed;
	// }
  // }

};  // class

MyRobotHWInterface::MyRobotHWInterface()
: running_(true)
  , private_nh("~")
  , start_srv_(nh.advertiseService("start", &MyRobotHWInterface::start_callback, this))
  , stop_srv_(nh.advertiseService("stop", &MyRobotHWInterface::stop_callback, this))
  {
    private_nh.param<double>("wheel_diameter", _wheel_diameter, 0.065);
    private_nh.param<double>("max_speed", _max_speed, 0.2);

    // Intialize raw data
    std::fill_n(pos, NUM_JOINTS, 0.0);
    std::fill_n(vel, NUM_JOINTS, 0.0);
    std::fill_n(eff, NUM_JOINTS, 0.0);
    std::fill_n(cmd, NUM_JOINTS, 0.0);

    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
      ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
    // connect and register the joint state and velocity interfaces
    for (unsigned int i = 0; i < NUM_JOINTS; ++i)
    {
      // std::ostringstream os;
      // os << "wheel_" << i << "_joint";

      hardware_interface::JointStateHandle state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
      jnt_state_interface.registerHandle(state_handle);

      hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(joint_names[i]), &cmd[i]);
      jnt_vel_interface.registerHandle(vel_handle);
    }
    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_vel_interface);

	// Initialize publishers and subscribers
	front_left_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("front_left_wheel_vel", 1);
  front_right_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("front_right_wheel_vel", 1);
  rear_left_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("rear_left_wheel_vel", 1);
	rear_right_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("rear_right_wheel_vel", 1);

	front_left_wheel_angle_sub_ = nh.subscribe("front_left_wheel", 1, &MyRobotHWInterface::frontLeftWheelAngleCallback, this);
  front_right_wheel_angle_sub_ = nh.subscribe("front_right_wheel", 1, &MyRobotHWInterface::frontRightWheelAngleCallback, this);
  rear_left_wheel_angle_sub_ = nh.subscribe("rear_left_wheel", 1, &MyRobotHWInterface::rearLeftWheelAngleCallback, this);
	rear_right_wheel_angle_sub_ = nh.subscribe("rear_right_wheel", 1, &MyRobotHWInterface::rearRightWheelAngleCallback, this);
}
