// interface
#include "sd_drone_plugin/sd_drone_plugin.hpp"
// gazebo
#include <gazebo/physics/physics.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
// other

namespace sd {

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

	this->ros2node_ = gazebo_ros::Node::Get(_sdf, _model);

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

	// curr pos publisher
	this->pos_pub_ =
		ros2node_->create_publisher<sd_interfaces::msg::PositionStamped>(
			"position", 1);

	// service server for setting target position
	this->target_service_ =
		this->ros2node_->create_service<sd_interfaces::srv::SetDroneTarget>(
			"set_target",
			std::bind(&DronePlugin::SetTargetCallback, this,
					  std::placeholders::_1, std::placeholders::_2));

	// callback each simulation step
	this->update_callback_ = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&DronePlugin::OnUpdate, this, std::placeholders::_1));

	// initial values
	this->pos_ = this->model_->WorldPose().Pos();
	this->target_pos_ = this->pos_;

	// INFO
	RCLCPP_INFO(
		ros2node_->get_logger(),
		std::string("Loaded SD Drone Plugin for Drone ID: " + this->id_ + " !")
			.c_str());
}

// called each iteration of simulation
void DronePlugin::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
	this->pos_ = this->model_->WorldPose().Pos();
	if ( this->motor_on_ ) {
		this->fakeRotation();
		this->flight_model();
	}

	this->publish_position();
	this->last_time_ = _info.simTime;
}

void DronePlugin::flight_model()
{

	{
		auto pose = this->model_->WorldPose();
		auto pos = pose.Pos();
		auto rot = pose.Rot();
		auto rpy = rot.Euler();

		rpy.X() = 0;
		rpy.Y() = 0;
		auto q = ignition::math::Quaterniond::EulerToQuaternion(rpy);
		pose.Rot() = q;
		this->model_->SetWorldPose(pose);

		auto pos_dif = this->target_pos_ - pos;
		auto rpy_diff = (ignition::math::Vector3d(0, 0, 0) - rpy) * 0.1;

		// target reached when dist < 0.1
		auto dist_vec = pos_dif;
		const auto distance = dist_vec.Length();

		// xy and zdistance
		const auto z_dist = dist_vec.Z();
		dist_vec.Z() = 0;
		const auto xy_dist = dist_vec.Length();

		// normalize the direction vector and multiply with max. velocity
		auto vel = pos_dif.Normalize() * this->max_vel_;
		if ( distance < 2 ) {
			vel *= distance / 2;
		}

		// compute the yaw orientation to target
		ignition::math::Angle yaw = atan2(pos_dif.Y(), pos_dif.X()) - rpy.Z();
		yaw.Normalize();

		// first lift up and rotate
		if ( z_dist > 0.3 || std::abs(yaw.Radian()) > IGN_DTOR(15) ) {
			// lif
			vel.X() = 0;
			vel.Y() = 0;
			this->model_->SetLinearVel(vel);
			rpy_diff.Z() = yaw.Radian();
			// rpy_diff = rpy_diff.Normalize();
			this->model_->SetAngularVel(rpy_diff);
			// rotate
			if ( xy_dist > 0.1 ) {
			}

		} else {
			this->model_->SetLinearVel(vel);
			rpy_diff.Z() = yaw.Radian();
			// rpy_diff = rpy_diff.Normalize();
			this->model_->SetAngularVel(rpy_diff);
		}
	}
}

ignition::math::Pose3d DronePlugin::getGaussianNoise() const
{
	return ignition::math::Pose3d(gaussian_noise(e2), gaussian_noise(e2),
								  gaussian_noise(e2), gaussian_noise(e2),
								  gaussian_noise(e2), gaussian_noise(e2));
}

// adds fake rotation to drone rotors
void DronePlugin::fakeRotation()
{
	constexpr double thrust = 0.5;
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

// publish current positiom to ros topic ../pos
void DronePlugin::publish_position() const
{
	sd_interfaces::msg::PositionStamped msg;
	msg.header.stamp = ros2node_->now();
	msg.header.frame_id = "world";

	msg.position.x = this->model_->WorldPose().X();
	msg.position.y = this->model_->WorldPose().Y();
	msg.position.z = this->model_->WorldPose().Z();

	this->pos_pub_->publish(msg);
}

void DronePlugin::SetTargetCallback(
	const std::shared_ptr<sd_interfaces::srv::SetDroneTarget::Request> request,
	std::shared_ptr<sd_interfaces::srv::SetDroneTarget::Response> response)
{
	this->target_pos_.X() = request->target.pos.x;
	this->target_pos_.Y() = request->target.pos.y;
	this->target_pos_.Z() = request->target.pos.z;
	this->motor_on_ = request->target.motors_on;

	response->set__success(true);
}

// Register this plugin
GZ_REGISTER_MODEL_PLUGIN(DronePlugin)

} // namespace sd