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
#include <cmath>
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

	this->rotor_thrust_coeff_ = 0.00025;
	this->rotor_torque_coeff_ = 0.0000074;

	// params
	if ( _sdf->HasElement("num_rotors") ) {
		this->num_rotors_ = _sdf->GetElement("num_rotors")->Get<uint>();
	} else {
		gzthrow("Please provide number of rotors !");
	}
	// for all rotors
	const std::string param_link_name_base = "link_name_rotor_";
	std::string param_link_name;
	for ( unsigned int i = 0; i < this->num_rotors_; i++ ) {
		// load link name
		param_link_name = param_link_name_base + std::to_string(i);
		if ( _sdf->HasElement(param_link_name) ) {
			this->rotor_link_names_.push_back(
				_sdf->GetElement(param_link_name)->Get<std::string>());
			this->rotor_rpms_.push_back(0);
		} else {
			gzthrow("Please provide link name of rotors !");
		}
	}

	// ros node + qos
	ros2node_ = gazebo_ros::Node::Get(_sdf, _parent);
	const gazebo_ros::QoS& qos = ros2node_->get_qos();

	// publisher
	// publisher_ = ros2node_->create_publisher<std_msgs::msg::String>(
	// 	"test_topic", qos.get_publisher_qos("test_topic", rclcpp::QoS(10)));

	// subscription
	subscription_ =
		ros2node_->create_subscription<sd_interfaces::msg::RotorRPM>(
			"test_topic2",
			qos.get_subscription_qos("test_topic2", rclcpp::QoS(1)),
			std::bind(&DronePlugin::topic_callback, this,
					  std::placeholders::_1));

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&DronePlugin::OnUpdate, this, std::placeholders::_1));

	// INFO
	RCLCPP_INFO(ros2node_->get_logger(), "Loaded DronePlugin!");
}

// called each iteration of simulation
void DronePlugin::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
	this->updateThrust();
}

// called each time receiving message from topic
void DronePlugin::topic_callback(
	const sd_interfaces::msg::RotorRPM::SharedPtr rotor_rpm)
{
	if ( rotor_rpm->rotor.size() == this->num_rotors_ ) {
		for ( unsigned short i = 0; i < rotor_rpm->rotor.size(); i++ ) {
			this->rotor_rpms_[i] = rotor_rpm->rotor[i].rpm;
		}
	} else {
		RCLCPP_ERROR(this->ros2node_->get_logger(),
					 "Provide RPM for all rotors!");
	}
}

void DronePlugin::updateThrust()
{
	double thrust;
	double torque;
	gazebo::physics::LinkPtr link;
	for ( unsigned int i = 0; i < this->num_rotors_; i++ ) {
		link = model_->GetLink(this->rotor_link_names_[i]);
		thrust =
			this->calculateThrust(this->rpm_to_rad_p_sec(this->rotor_rpms_[i]));
		torque =
			this->calculateTorque(this->rpm_to_rad_p_sec(this->rotor_rpms_[i]));
		if ( link != NULL ) {
			link->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust));
			link->AddRelativeTorque(ignition::math::Vector3d(0, 0, torque));
		}
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

double DronePlugin::rpm_to_rad_p_sec(const int& rpm)
{
	return rpm / 60 * 2 * M_PI;
}

// Register this plugin
GZ_REGISTER_MODEL_PLUGIN(DronePlugin)

} // namespace gazebo_ros_plugins
} // namespace sd