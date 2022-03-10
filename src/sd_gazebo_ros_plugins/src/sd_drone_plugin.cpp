// interface
#include "sd_gazebo_ros_plugins/sd_simple_drone_plugin.hpp"
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
SimpleDronePlugin::SimpleDronePlugin()
{
}

SimpleDronePlugin ::~SimpleDronePlugin()
{
}

void SimpleDronePlugin::Load(gazebo::physics::ModelPtr _model,
							 sdf::ElementPtr _sdf)
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

	// max vel
	if ( _sdf->HasElement("max_vel") ) {
		this->max_vel_ =
			_sdf->GetElement("max_vel")->Get<ignition::math::Vector3d>();
	} else {
		this->max_vel_ = ignition::math::Vector3d(1, 1, 1);
	}

	// max angular vel
	if ( _sdf->HasElement("max_ang_vel") ) {
		this->max_ang_vel_ =
			_sdf->GetElement("max_ang_vel")->Get<ignition::math::Vector3d>();
	} else {
		this->max_ang_vel_ = ignition::math::Vector3d(0.1, 0.1, 0.1);
	}

	// curr pose publisher
	const std::string pose_pub_name = this->model_->GetName() + "/curr_pose";
	this->pose_pub_ =
		ros2node_->create_publisher<geometry_msgs::msg::PoseStamped>(
			pose_pub_name, 10);

	// goal pose subscription
	const std::string rpm_sub_name = this->model_->GetName() + "/goal_pose";
	this->pose_sub_ =
		ros2node_->create_subscription<geometry_msgs::msg::PoseStamped>(
			rpm_sub_name, 1,
			std::bind(&SimpleDronePlugin::on_pose_msg_callback, this,
					  std::placeholders::_1));

	// callback each simulation step
	this->update_callback_ = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&SimpleDronePlugin::OnUpdate, this, std::placeholders::_1));

	// INFO
	RCLCPP_INFO(ros2node_->get_logger(), "Loaded SD Drone Plugin!");
}

// called each iteration of simulation
void SimpleDronePlugin::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
	this->last_time_ = _info.simTime;
	this->pose_ = this->model_->WorldPose();

	this->fakeRotation();
	this->fakeFly();
	this->publish_pose();
}

void SimpleDronePlugin::fakeFly()
{
	const auto scalar =
		ignition::math::Pose3d(0.001, 0.001, 0.001, 0.001, 0.001, 0.001);
	auto vel_raw = scalar * (this->goal_pose_ - this->pose_);
	this->vel_ = vel_raw.Pos();
	this->ang_vel_ = vel_raw.Rot().Euler();

	this->cropVelocity();
	this->model_->SetLinearVel(this->vel_);
	this->model_->SetAngularVel(this->ang_vel_);
}

void SimpleDronePlugin::cropVelocity()
{
	// vel
	if ( this->vel_.X() > this->max_vel_.X() ) {
		this->vel_.X() = this->max_vel_.X();
	}
	if ( this->vel_.Y() > this->max_vel_.Y() ) {
		this->vel_.Y() = this->max_vel_.Y();
	}
	if ( this->vel_.Z() > this->max_vel_.Z() ) {
		this->vel_.Z() = this->max_vel_.Z();
	}
	// angular vel
	if ( this->ang_vel_.X() > this->max_ang_vel_.X() ) {
		this->ang_vel_.X() = this->max_ang_vel_.X();
	}
	if ( this->ang_vel_.Y() > this->max_ang_vel_.Y() ) {
		this->ang_vel_.Y() = this->max_ang_vel_.Y();
	}
	if ( this->ang_vel_.Z() > this->max_ang_vel_.Z() ) {
		this->ang_vel_.Z() = this->max_ang_vel_.Z();
	}
}

void SimpleDronePlugin::publish_pose() const
{
	geometry_msgs::msg::PoseStamped msg;
	msg.header.stamp = ros2node_->now();
	msg.header.frame_id = this->model_->GetName();

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
void SimpleDronePlugin::on_pose_msg_callback(
	const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	this->goal_pose_.SetX(msg->pose.position.x);
	this->goal_pose_.SetY(msg->pose.position.y);
	this->goal_pose_.SetZ(msg->pose.position.z);

	this->goal_pose_.Rot().X() = msg->pose.orientation.x;
	this->goal_pose_.Rot().Y() = msg->pose.orientation.y;
	this->goal_pose_.Rot().Z() = msg->pose.orientation.z;
	this->goal_pose_.Rot().W() = msg->pose.orientation.w;
}

void SimpleDronePlugin::fakeRotation()
{
	constexpr double thrust = 5;
	int sign = 1;
	gazebo::physics::LinkPtr link;
	// apply fake rotation for each link
	for ( unsigned int i = 0; i < this->num_rotors_; i++ ) {
		link = model_->GetLink(this->rotor_link_names_[i]);
		if ( link != NULL ) {
			link->AddRelativeTorque(
				ignition::math::Vector3d(0, 0, sign * thrust));
			// link->AddRelativeForce(ignition::math::Vector3d(0, 0, 4.70));
			sign *= -1;
		}
	}
}

// Register this plugin
GZ_REGISTER_MODEL_PLUGIN(SimpleDronePlugin)

} // namespace gazebo_ros_plugins
} // namespace sd