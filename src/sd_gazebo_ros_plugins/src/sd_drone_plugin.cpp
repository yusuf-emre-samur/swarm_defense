// interface
#include "sd_gazebo_ros_plugins/sd_drone_plugin.hpp"
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
DronePlugin::DronePlugin()
{
}

DronePlugin ::~DronePlugin()
{
}

void DronePlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	// Store the pointer to the model
	this->model_ = _parent;

	// ros node + qos
	ros2node_ = gazebo_ros::Node::Get(_sdf, _parent);
	const gazebo_ros::QoS& qos = ros2node_->get_qos();

	// params
	if ( _sdf->HasElement("rotor_0") ) {
		this->rotor_link_names_.push_back(
			_sdf->GetElement("rotor_0")->Get<std::string>());
	} else {
		this->rotor_link_names_.push_back("rotor_0");
	}
	if ( _sdf->HasElement("rotor_1") ) {
		this->rotor_link_names_.push_back(
			_sdf->GetElement("rotor_1")->Get<std::string>());
	} else {
		this->rotor_link_names_.push_back("rotor_0");
	}
	if ( _sdf->HasElement("rotor_2") ) {
		this->rotor_link_names_.push_back(
			_sdf->GetElement("rotor_2")->Get<std::string>());
	} else {
		this->rotor_link_names_.push_back("rotor_0");
	}
	if ( _sdf->HasElement("rotor_3") ) {
		this->rotor_link_names_.push_back(
			_sdf->GetElement("rotor_3")->Get<std::string>());
	} else {
		this->rotor_link_names_.push_back("rotor_0");
	}

	// publisher
	publisher_ = ros2node_->create_publisher<std_msgs::msg::String>(
		"test_topic", qos.get_publisher_qos("test_topic", rclcpp::QoS(10)));

	// subscription
	subscription_ = ros2node_->create_subscription<geometry_msgs::msg::Vector3>(
		"test_topic2", qos.get_subscription_qos("test_topic2", rclcpp::QoS(1)),
		std::bind(&DronePlugin::topic_callback, this, std::placeholders::_1));

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&DronePlugin::OnUpdate, this));

	// INFO
	RCLCPP_INFO(ros2node_->get_logger(), "Loaded DronePlugin!");
}

// called each iteration of simulation
void DronePlugin::OnUpdate()
{
	std::unique_lock<std::mutex> lock(this->pose_mtx_);
	this->pose_ = this->model_->WorldPose();
	lock.unlock();
	this->updateThrust();
	// create msg
	auto msg = std_msgs::msg::String();
	msg.data = "Hello World ";
	// publish
	this->publisher_->publish(msg);
}

// called each time receiving message from topic
void DronePlugin::topic_callback(
	const geometry_msgs::msg::Vector3::SharedPtr msg)
{
	RCLCPP_INFO(this->ros2node_->get_logger(), std::to_string(msg->x).c_str());
}

void DronePlugin::updateThrust()
{
	for ( const std::string& rotor_link_name : this->rotor_link_names_ ) {
		RCLCPP_INFO(this->ros2node_->get_logger(), rotor_link_name.c_str());
	}
}

double DronePlugin::calculateThrust(const double& w)
{
	double thrust = rotor_thrust_coeff_ * w * w;
	return thrust;
}

double DronePlugin::calculateTorque(const double& w)
{
	double torque = copysign(rotor_torque_coeff_ * w * w, w);
	return torque;
}

// Register this plugin
GZ_REGISTER_MODEL_PLUGIN(DronePlugin)

} // namespace gazebo_ros_plugins
} // namespace sd