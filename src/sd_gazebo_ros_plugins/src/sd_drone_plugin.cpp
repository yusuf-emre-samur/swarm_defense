// interface
#include "sd_gazebo_ros_plugins/sd_drone_plugin.hpp"
// gazebo
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Quaternion.hh>
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

// Constructor
DronePlugin::DronePlugin()
{
}

DronePlugin ::~DronePlugin()
{
}

void DronePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

	this->model_ = _model;
	this->ros2node_ = gazebo_ros::Node::Get();

	// rotor link names
	const std::string param_link_name_base = "link_name_rotor_";
	std::string param_link_name;
	for ( unsigned int i = 0; i < this->num_rotors_; i++ ) {
		param_link_name = param_link_name_base + std::to_string(i);
		if ( _sdf->HasElement(param_link_name) ) {
			this->rotor_link_names_[i] =
				_sdf->GetElement(param_link_name)->Get<std::string>();
		} else {
			gzthrow("Please provide link name of rotors !");
		}
	}

	// imu sensor
	std::string imu_link_name, imu_sensor_name;
	if ( _sdf->HasElement("imu_link_name") ) {
		imu_link_name = _sdf->GetElement("imu_link_name")->Get<std::string>();
	} else {
		gzthrow("Please provide imu_link_name");
	}
	if ( _sdf->HasElement("imu_sensor_name") ) {
		imu_sensor_name =
			_sdf->GetElement("imu_sensor_name")->Get<std::string>();
	} else {
		gzthrow("Please provide imu_sensor_name");
	}
	// imu raw sensor
	auto imu_sensor_ = gazebo::sensors::get_sensor(
		this->model_->GetWorld()->Name() +
		"::" + this->model_->GetScopedName() + "::" + imu_link_name +
		"::" + imu_sensor_name);
	// imu sensor
	if ( imu_sensor_.get() == nullptr ) {
		gzthrow("Could not find imu sensor, check parameter!");
	} else {
		this->imu_ =
			std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(imu_sensor_);
	}

	// pose publisher
	const std::string pose_pub_name = this->model_->GetName() + "/pose";
	this->pose_pub_ =
		ros2node_->create_publisher<geometry_msgs::msg::PoseStamped>(
			pose_pub_name, 10);

	// rpm subscription
	const std::string rpm_sub_name = this->model_->GetName() + "/rpm";
	this->rpm_sub_ =
		ros2node_->create_subscription<sd_interfaces::msg::QuadcopterRPM>(
			rpm_sub_name, 1,
			std::bind(&DronePlugin::on_msg_rpm_callback, this,
					  std::placeholders::_1));

	// callback each simulation step
	this->update_callback_ = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&DronePlugin::OnUpdate, this, std::placeholders::_1));

	// INFO
	RCLCPP_INFO(ros2node_->get_logger(), "Loaded SD Drone Plugin!");
}

// called each iteration of simulation
void DronePlugin::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
	this->updateThrust();
	this->publish_pose();
	this->last_time_ = _info.simTime;
}

void DronePlugin::publish_pose() const
{
	geometry_msgs::msg::PoseStamped msg;
	msg.header.stamp = ros2node_->now();
	msg.header.frame_id = this->imu_->ScopedName();

	msg.pose.orientation.x = this->imu_->Orientation().X();
	msg.pose.orientation.y = this->imu_->Orientation().Y();
	msg.pose.orientation.z = this->imu_->Orientation().Z();
	msg.pose.orientation.w = this->imu_->Orientation().W();

	msg.pose.position.x = this->model_->WorldPose().X();
	msg.pose.position.y = this->model_->WorldPose().Y();
	msg.pose.position.z = this->model_->WorldPose().Z();

	this->pose_pub_->publish(msg);
}

// called each time receiving message from topic
void DronePlugin::on_msg_rpm_callback(
	const sd_interfaces::msg::QuadcopterRPM::SharedPtr rotor_rpm)
{
	// update rpm
	this->rotor_rpms_[0] = rotor_rpm->rotor0.rpm;
	this->rotor_rpms_[1] = rotor_rpm->rotor1.rpm;
	this->rotor_rpms_[2] = rotor_rpm->rotor2.rpm;
	this->rotor_rpms_[3] = rotor_rpm->rotor3.rpm;
}

void DronePlugin::updateThrust()
{
	double thrust;
	double torque;
	gazebo::physics::LinkPtr link;

	for ( unsigned int i = 0; i < this->num_rotors_; i++ ) {
		link = model_->GetLink(this->rotor_link_names_[i]);

		// calc. and apply force and torque
		if ( link != NULL ) {
			thrust =
				this->calculateThrust(this->rpm_to_rad(this->rotor_rpms_[i]));
			torque =
				this->calculateTorque(this->rpm_to_rad(this->rotor_rpms_[i]));
			link->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust));
			link->AddRelativeTorque(ignition::math::Vector3d(0, 0, torque));
		}
	}
}

double DronePlugin::calculateThrust(const double& w) const
{
	double thrust = rotor_thrust_coeff_ * w * w;
	return thrust;
}

double DronePlugin::calculateTorque(const double& w) const
{
	double torque = copysign(rotor_torque_coeff_ * w * w, w);
	return torque;
}

double DronePlugin::rpm_to_rad(const int& rpm) const
{
	return rpm / 60 * 2 * M_PI;
}

// Register this plugin
GZ_REGISTER_MODEL_PLUGIN(DronePlugin)

} // namespace gazebo_ros_plugins
} // namespace sd