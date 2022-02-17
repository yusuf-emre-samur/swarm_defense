// interface
#include "sd_gazebo_ros_plugins/ros_test_plugin.hpp"
// gazebo
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
// ros
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rclcpp/rclcpp.hpp>
// other
#include <memory>
#include <string>

namespace sd {
namespace gazebo_ros_plugins {

using namespace std::chrono_literals;

// Constructor
RosTestPlugin::RosTestPlugin()
{
}

RosTestPlugin ::~RosTestPlugin()
{
}

void RosTestPlugin::Load(gazebo::physics::ModelPtr _parent,
						 sdf::ElementPtr _sdf)
{
	// Store the pointer to the model
	this->model_ = _parent;

	// ros node + qos
	ros2node_ = gazebo_ros::Node::Get(_sdf, _parent);
	const gazebo_ros::QoS& qos = ros2node_->get_qos();

	// publisher
	publisher_ = ros2node_->create_publisher<std_msgs::msg::String>(
		"test_topic", qos.get_publisher_qos("test_topic", rclcpp::QoS(10)));

	// subscription
	subscription_ = ros2node_->create_subscription<geometry_msgs::msg::Vector3>(
		"test_topic2", qos.get_subscription_qos("test_topic2", rclcpp::QoS(1)),
		std::bind(&RosTestPlugin::topic_callback, this, std::placeholders::_1));

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&RosTestPlugin::OnUpdate, this));

	// INFO
	RCLCPP_INFO(ros2node_->get_logger(), "Loaded RosTestPlugin!");
}

// called each iteration of simulation
void RosTestPlugin::OnUpdate()
{
	// create msg
	auto msg = std_msgs::msg::String();
	msg.data = "Hello World ";
	// publish
	publisher_->publish(msg);
}

// called each time receiving message from topic
void RosTestPlugin::topic_callback(
	const geometry_msgs::msg::Vector3::SharedPtr msg)
{
	RCLCPP_INFO(ros2node_->get_logger(), std::to_string(msg->x).c_str());
}

// Register this plugin
GZ_REGISTER_MODEL_PLUGIN(RosTestPlugin)

} // namespace gazebo_ros_plugins
} // namespace sd