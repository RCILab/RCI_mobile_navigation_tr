/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
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
 *   * Neither the name of the Neobotix nor the names of its
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

#include "../../common/include/Kinematics.h"
#include "../../common/include/DiffDrive2WKinematics.h"
#include "rclcpp/rclcpp.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;


class PlatformCtrlNode: public rclcpp::Node
{
public:
	PlatformCtrlNode(): Node("tr_differential_node") {}

	int init() {
		this->declare_parameter<double>("wheelDiameter", 0.3);
		this->declare_parameter<double>("wheelSeperation", 0.5);
		this->declare_parameter<std::string>("odomFrame", "odom");
		this->declare_parameter<std::string>("robotBaseFrame", "base_footprint");
		this->declare_parameter<std::string>("joint_trajectory_topic", "drives/joint_trajectory");
		this->declare_parameter<std::string>("joint_states_topic", "drives/joint_states");

		this->get_parameter("wheelDiameter", wheelDiameter);
		this->get_parameter("wheelSeperation", axisLength);
		this->get_parameter("odomFrame", odomFrame);
		this->get_parameter("robotBaseFrame", robotBaseFrame);
		this->get_parameter("joint_trajectory_topic", joint_trajectory_topic);
		this->get_parameter("joint_states_topic", joint_states_topic);

		topicPub_Odometry = this->create_publisher<nav_msgs::msg::Odometry>("odom1", 1000);
    	topicPub_DriveCommands = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_trajectory_topic, 1000);
		topicSub_ComVel = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&PlatformCtrlNode::receiveCmd, this, _1));
		topicSub_DriveState = this->create_subscription<sensor_msgs::msg::JointState>(joint_states_topic, 10, std::bind(&PlatformCtrlNode::receiveOdo, this, _1));
		odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		DiffDrive2WKinematics* diffKin = new DiffDrive2WKinematics();
		diffKin->setWheelDiameter(wheelDiameter);
		diffKin->setAxisLength(axisLength);
		kin = diffKin;

		RCLCPP_INFO(this->get_logger(), "Kinematics node initialized.");
		RCLCPP_INFO(this->get_logger(), "Subscribing to joint states on: %s", joint_states_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Publishing joint commands to: %s", joint_trajectory_topic.c_str());

		return 0;
	}

	void receiveCmd(const geometry_msgs::msg::Twist::SharedPtr twist) {
		trajectory_msgs::msg::JointTrajectory traj;
		kin->execInvKin(twist, traj);
		topicPub_DriveCommands->publish(traj);
	}

	void receiveOdo(const sensor_msgs::msg::JointState::SharedPtr js) {

		nav_msgs::msg::Odometry odom;
		odom.header.frame_id = odomFrame;
		odom.child_frame_id = robotBaseFrame;
		kin->execForwKin(js, odom);
		topicPub_Odometry->publish(odom);


		//odometry transform:
		if(sendTransform) {
			std::string robot_namespace(this->get_namespace());
			geometry_msgs::msg::TransformStamped odom_trans;
			odom_trans.header.stamp = odom.header.stamp;
			if(robot_namespace != "/") {
				robot_namespace.erase(
					std::remove(robot_namespace.begin(),
					robot_namespace.end(),
					'/'), robot_namespace.end());
				odom_trans.header.frame_id = robot_namespace + odomFrame;
				odom_trans.child_frame_id = robot_namespace + robotBaseFrame;
			} else {
				odom_trans.header.frame_id = odomFrame;
				odom_trans.child_frame_id = robotBaseFrame;
			}

			odom_trans.transform.translation.x = odom.pose.pose.position.x;
			odom_trans.transform.translation.y = odom.pose.pose.position.y;
			odom_trans.transform.translation.z = odom.pose.pose.position.z;
			odom_trans.transform.rotation = odom.pose.pose.orientation;
			odom_broadcaster->sendTransform(odom_trans);
		}
	}


private:
	std::mutex m_node_mutex;
	Kinematics* kin = 0;
	bool sendTransform = true;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr topicPub_Odometry;
	rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr topicPub_DriveCommands;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr topicSub_ComVel;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topicSub_DriveState;
	std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

	double wheelDiameter = 0.0;
	double axisWidth = 0.0;
	double axisLength = 0.0;

	std::string odomFrame;
	std::string robotBaseFrame;
    std::string joint_trajectory_topic;
    std::string joint_states_topic;
};

int main (int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto nh = std::make_shared<PlatformCtrlNode>();
	if(nh->init() != 0) {
			RCLCPP_ERROR_STREAM(nh->get_logger(),"tr_kinematics_differential_node: init failed!");
		}
	rclcpp::spin(nh);

	return 0;
}
