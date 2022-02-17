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
#include <rclcpp/rclcpp.hpp>

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
	this->model = _parent;

	ros2node_ = gazebo_ros::Node::Get(_sdf, _parent);
	const gazebo_ros::QoS& qos = ros2node_->get_qos();

	publisher_ = ros2node_->create_publisher<std_msgs::msg::String>(
		"test_topic", qos.get_publisher_qos("test_topic", rclcpp::QoS(10)));

	// timer_ = ros2node_->create_wall_timer(
	// 	500ms, std::bind(&RosTestPlugin::timer_callback, ros2node_.get()));

	RCLCPP_INFO(ros2node_->get_logger(), "Ros Test Plugin Load");

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&RosTestPlugin::OnUpdate, this));
}

// Called by the world update start event
void RosTestPlugin::OnUpdate()
{
	auto msg = std_msgs::msg::String();
	msg.data = "Hello World ";
	publisher_->publish(msg);

	// Apply a small linear velocity to the model.
	this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RosTestPlugin)

} // namespace gazebo_ros_plugins
} // namespace sd