#include "sd_drone_controller/sd_drone_controller.hpp"

namespace sd {
DroneController::DroneController() : rclcpp::Node("DroneController")
{
	// parameters
	// drone id
	this->declare_parameter<uint8_t>("drone_id", 0);
	this->get_parameter("drone_id", this->id_);
	this->name_ = "drone_" + std::to_string(this->id_);

	// drone base station
	this->declare_parameter<std::vector<double>>("base_station_pos");
	rclcpp::Parameter base_station_pos("base_station_pos",
									   std::vector<double>({}));
	this->get_parameter("base_station_pos", base_station_pos);
	auto tmp = base_station_pos.as_double_array();
	this->base_station_pos_.x() = tmp[0];
	this->base_station_pos_.y() = tmp[1];
	this->base_station_pos_.z() = tmp[2];

	this->drone_mode_ = DroneMode::READY;
	this->flight_mode_ = FlightMode::LANDED;

	this->battery_ = 100.0;

	using namespace std::chrono_literals;
	this->timer_ =
		rclcpp::create_timer(this, this->get_clock(), 250ms,
							 std::bind(&DroneController::timer_callback, this));

	// ros subscriber
	this->sub_world_objects_ =
		this->create_subscription<sd_interfaces::msg::WorldObjects>(
			"/world/objects", 1,
			std::bind(&DroneController::callback_world_objects, this,
					  std::placeholders::_1));

	this->sub_comm_receive_ =
		this->create_subscription<sd_interfaces::msg::SwarmInfo>(
			"communication_receive", 1,
			std::bind(&DroneController::callback_comm_receive, this,
					  std::placeholders::_1));

	this->sub_position_ =
		this->create_subscription<sd_interfaces::msg::PositionStamped>(
			"position", 1,
			std::bind(&DroneController::callback_position, this,
					  std::placeholders::_1));

	this->pub_comm_send_ = create_publisher<sd_interfaces::msg::DroneMsgOut>(
		"communication_send", 1);

	// ros publisher
	this->pub_target_ =
		this->create_publisher<sd_interfaces::msg::FlightTarget>("target", 1);
}

void DroneController::timer_callback()
{

	this->battery_ = this->battery_ - (1.0 / 60.0); // (1 / 4) * (1 / 15);
	RCLCPP_INFO(this->get_logger(), std::to_string(this->battery_).c_str());
	this->check_swarm_information();

	// this->detect_threats();
	// this->filter_detected_threats();
	// this->calculate_pso_velocity();
	switch ( this->flight_mode_ ) {
	case FlightMode::FLYING:
		this->publish_target();
		break;

	case FlightMode::LANDED:
		this->drone_landed();

		break;
	case FlightMode::LADNING:
		this->drone_landing();
		this->publish_target();

		break;

	case FlightMode::STARTING:
		this->drone_start();
		this->publish_target();
		break;

	default:
		break;
	}

	switch ( this->drone_mode_ ) {
	case DroneMode::FLYING:

		break;

	default:
		break;
	}

	this->send_message_to_swarm();

	this->last_time_ = this->now();
}

void DroneController::check_swarm_information()
{
	bool drone_in_air = this->flight_mode_ == FlightMode::STARTING ||
						this->flight_mode_ == FlightMode::FLYING;
	bool drone_landing = this->flight_mode_ == FlightMode::LADNING ||
						 this->flight_mode_ == FlightMode::LANDED;
	bool swarm_needs_drone = this->has_to_start();
	if ( swarm_needs_drone && !drone_in_air ) {
		this->flight_mode_ = FlightMode::STARTING;
	}
	if ( !swarm_needs_drone && !drone_landing ) {
		this->flight_mode_ = FlightMode::LADNING;
	}
}

bool DroneController::has_to_start() const
{
	// check how many drones are in flying mode
	uint8_t num_drones_flying = 0;
	uint8_t num_lower_id_drones = 0;
	DroneMode drone_mode;
	FlightMode flight_mode;
	for ( const auto& drone : this->swarm_info_.swarm_positions.drones ) {
		drone_mode = static_cast<DroneMode>(drone.drone_mode);
		flight_mode = static_cast<FlightMode>(drone.flight_mode);
		if ( drone_mode == DroneMode::FLYING ) {
			++num_drones_flying;
		}
		if ( drone.drone_id < this->id_ ) {
			++num_lower_id_drones;
		}
	}
	if ( num_drones_flying <= 2 && num_lower_id_drones <= 1 ) {
		return true;
		// if ( this->flight_mode_ != FlightMode::STARTING ||
		// 	 this->flight_mode_ != FlightMode::FLYING ) {
		// 	return true;
		// }

	} else {
		// if ( this->flight_mode_ != FlightMode::LANDED ) {
		// 	this->flight_mode_ = FlightMode::LADNING;
		// }
		return false;
	}
}

void DroneController::drone_start()
{

	auto target = this->base_station_pos_ + Eigen::Vector3d(0, 0, 10);
	auto dist = (this->position_ - target).cwiseAbs().norm();
	if ( dist < 0.05 ) {
		this->flight_mode_ = FlightMode::FLYING;
		this->drone_mode_ = DroneMode::FLYING;
	} else {
		this->target_ = target;
	}
}

void DroneController::drone_start_prep()
{
}

void DroneController::drone_landed()
{
	// turn motors off
	sd_interfaces::msg::FlightTarget target;
	target.motors_on = false;
	this->pub_target_->publish(target);
}

void DroneController::drone_landing()
{
	auto dist = (this->position_ - this->base_station_pos_).cwiseAbs().norm();
	this->drone_mode_ = DroneMode::READY;
	if ( dist < 0.05 ) {
		this->flight_mode_ = FlightMode::LANDED;
	} else {
		this->flight_mode_ = FlightMode::LADNING;
		this->target_ = this->base_station_pos_;
	}
}

void DroneController::turn_motors_off()
{
	sd_interfaces::msg::FlightTarget target;
	target.motors_on = false;

	this->pub_target_->publish(target);
}

void DroneController::detect_threats()
{
}

void DroneController::filter_detected_threats()
{
}

void DroneController::calculate_pso_velocity()
{
}

void DroneController::publish_target()
{
	this->publish_target(this->target_);
}

void DroneController::publish_target(const Eigen::Vector3d& pos)
{
	sd_interfaces::msg::FlightTarget target;
	target.pos.x = pos.x();
	target.pos.y = pos.y();
	target.pos.z = pos.z();
	target.motors_on = true;

	this->pub_target_->publish(target);
}

void DroneController::flight_to_base_station()
{
	this->publish_target(this->base_station_pos_);
}

void DroneController::send_message_to_swarm()
{
	sd_interfaces::msg::DroneMsgOut msg;
	// header
	msg.drone_header.drone_id = this->id_;
	msg.drone_header.drone_mode = static_cast<uint8_t>(this->drone_mode_);
	msg.drone_header.flight_mode = static_cast<uint8_t>(this->flight_mode_);
	msg.drone_header.stamp = this->now();
	// pos
	msg.drone_header.pos.x = this->position_.x();
	msg.drone_header.pos.y = this->position_.y();
	msg.drone_header.pos.z = this->position_.z();

	this->pub_comm_send_->publish(msg);
}

// subscirber world objects
void DroneController::callback_world_objects(
	const sd_interfaces::msg::WorldObjects::SharedPtr msg)
{
}

// subscriber communincation receive
void DroneController::callback_comm_receive(
	const sd_interfaces::msg::SwarmInfo& msg)
{
	this->swarm_info_ = msg;
}

// subscriber position
void DroneController::callback_position(
	const sd_interfaces::msg::PositionStamped::SharedPtr msg)
{
	this->position_.x() = msg->position.x;
	this->position_.y() = msg->position.y;
	this->position_.z() = msg->position.z;
}

} // namespace sd

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sd::DroneController>());
	rclcpp::shutdown();
	return 0;
}
