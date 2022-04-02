// interface
#include "sd_gazebo_ros_plugins/sd_drone_plugin.hpp"
// gazebo
#include <gazebo/physics/physics.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
// other

namespace sd {
namespace gazebo_ros_plugins {

// noise generator
std::random_device rd{};
std::mt19937 e2{rd()};
std::normal_distribution<> gaussian_noise(0, 0.001);

// Constructor
DronePlugin::DronePlugin()
{
}

DronePlugin::~DronePlugin()
{
}

void DronePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	this->model_ = _model;
	this->id_ = this->model_->GetName();
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
	const std::string pose_pub_name = this->id_ + "/pos";
	this->pos_pub_ =
		ros2node_->create_publisher<sd_interfaces::msg::Position3Stamped>(
			pose_pub_name, 10);

	// goal pose subscription
	const std::string rpm_sub_name = this->id_ + "/target_pos";
	this->pos_sub_ =
		ros2node_->create_subscription<sd_interfaces::msg::Position3Stamped>(
			rpm_sub_name, 1,
			std::bind(&DronePlugin::on_position_msg_callback, this,
					  std::placeholders::_1));

	// callback each simulation step
	this->update_callback_ = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&DronePlugin::OnUpdate, this));

	this->pos_ = this->model_->WorldPose().Pos();
	this->target_pos_ = this->pos_;

	// INFO
	RCLCPP_INFO(ros2node_->get_logger(), "Loaded SD Drone Plugin!");
}

// called each iteration of simulation
void DronePlugin::OnUpdate()
{
	this->pos_ = this->model_->WorldPose().Pos();

	this->fakeFly();
	// this->gimbal();
	this->publish_position();
}

void DronePlugin::fakeFly()
{
	const auto q = ignition::math::Quaterniond::EulerToQuaternion(
		ignition::math::Vector3d(0, 0, 0));
	const auto scalar =
		ignition::math::Pose3d(0.001, 0.001, 0.001, 0.001, 0.001, 0.001);
	auto pose = this->model_->WorldPose();
	auto target_pose = ignition::math::Pose3d(this->target_pos_, q);

	auto vel_raw = scalar * (target_pose + this->getGaussianNoise() - pose);
	// angular velocity
	this->ang_vel_ = vel_raw.Rot().Euler() + this->fakeAngularMotion();
	// fix motion direction of angle
	vel_raw.CoordRotationSub(
		ignition::math::Quaterniond::EulerToQuaternion(ang_vel_));
	this->vel_ = vel_raw.Pos();
	this->cropVelocity();

	this->model_->SetLinearVel(this->vel_);
	this->model_->SetAngularVel(this->ang_vel_);
	if ( this->pos_.Z() > 0.001 ) {
		this->fakeRotation();
	}
	this->last_vel_ = this->vel_;
}

ignition::math::Pose3d DronePlugin::getGaussianNoise() const
{
	return ignition::math::Pose3d(gaussian_noise(e2), gaussian_noise(e2),
								  gaussian_noise(e2), gaussian_noise(e2),
								  gaussian_noise(e2), gaussian_noise(e2));
}

// add roll on y motion and pitch on x motion for realistic fly
ignition::math::Vector3d DronePlugin::fakeAngularMotion() const
{
	const double x_vel_pitch = -(this->vel_.Y() / this->max_vel_.Y() / 10);
	const double y_vel_roll = this->vel_.X() / this->max_vel_.X() / 10;
	return ignition::math::Vector3d(x_vel_pitch, y_vel_roll, 0);
}

// crop velocity and angular velocity to [-max, max]
void DronePlugin::cropVelocity()
{
	auto negative_max_vel = -1 * this->max_vel_;
	auto negative_max_ang_vel = -1 * this->max_ang_vel_;
	// x
	if ( this->vel_.X() > 0 && this->vel_.X() > this->max_vel_.X() ) {
		this->vel_.X() = this->max_vel_.X();
	}
	if ( this->vel_.X() < 0 && this->vel_.X() < negative_max_vel.X() ) {
		this->vel_.X() = negative_max_vel.X();
	}
	// y
	if ( this->vel_.Y() > 0 && this->vel_.Y() > this->max_vel_.Y() ) {
		this->vel_.Y() = this->max_vel_.Y();
	}
	if ( this->vel_.Y() < 0 && this->vel_.Y() < negative_max_vel.Y() ) {
		this->vel_.Y() = negative_max_vel.Y();
	}
	// y
	if ( this->vel_.Z() > 0 && this->vel_.Z() > this->max_vel_.Z() ) {
		this->vel_.Z() = this->max_vel_.Z();
	}
	if ( this->vel_.Z() < 0 && this->vel_.Z() < negative_max_vel.Z() ) {
		this->vel_.Z() = negative_max_vel.Z();
	}

	// r
	if ( this->ang_vel_.X() > 0 && this->ang_vel_.X() > this->max_vel_.X() ) {
		this->ang_vel_.X() = this->max_vel_.X();
	}
	if ( this->ang_vel_.X() < 0 &&
		 this->ang_vel_.X() < negative_max_ang_vel.X() ) {
		this->ang_vel_.X() = negative_max_ang_vel.X();
	}
	// p
	if ( this->ang_vel_.Y() > 0 && this->ang_vel_.Y() > this->max_vel_.Y() ) {
		this->ang_vel_.Y() = this->max_vel_.Y();
	}
	if ( this->ang_vel_.Y() < 0 &&
		 this->ang_vel_.Y() < negative_max_ang_vel.Y() ) {
		this->ang_vel_.Y() = negative_max_ang_vel.Y();
	}
	// y
	if ( this->ang_vel_.Z() > 0 && this->ang_vel_.Z() > this->max_vel_.Z() ) {
		this->ang_vel_.Z() = this->max_vel_.Z();
	}
	if ( this->ang_vel_.Z() < 0 &&
		 this->ang_vel_.Z() < negative_max_ang_vel.Z() ) {
		this->ang_vel_.Z() = negative_max_ang_vel.Z();
	}
}

// adds fake rotation to drone rotors
void DronePlugin::fakeRotation()
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

// update target position on new msg to ros topic ../target_pos
void DronePlugin::on_position_msg_callback(
	const sd_interfaces::msg::Position3Stamped::SharedPtr msg)
{
	this->target_pos_.X() = msg->pos3.x;
	this->target_pos_.Y() = msg->pos3.y;
	this->target_pos_.Z() = msg->pos3.z;
}

// publish current positiom to ros topic ../pos
void DronePlugin::publish_position() const
{
	sd_interfaces::msg::Position3Stamped msg;
	msg.header.stamp = ros2node_->now();
	msg.header.frame_id = "world";

	msg.pos3.x = this->model_->WorldPose().X();
	msg.pos3.y = this->model_->WorldPose().Y();
	msg.pos3.z = this->model_->WorldPose().Z();

	this->pos_pub_->publish(msg);
}

// Register this plugin
GZ_REGISTER_MODEL_PLUGIN(DronePlugin)

} // namespace gazebo_ros_plugins
} // namespace sd