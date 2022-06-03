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

	// starting battery percentage
	this->declare_parameter<double>("battery", 100.0);
	this->get_parameter("battery", this->battery_);

	if ( this->battery_ < 80.0 ) {
		// start at e
		this->drone_mode_ = DroneMode::NOTREADY;
	} else {
		// start at a
		this->drone_mode_ == DroneMode::READY;
	}
	this->flight_mode_ = FlightMode::LANDED;

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
	this->send_message_to_swarm();
	this->simulate_battery();
	this->check_swarm_information();
	this->flight();
	// this->detect_threats();
	// this->filter_detected_threats();
	// this->calculate_pso_velocity();

	this->last_time_ = this->now();
}

void DroneController::simulate_battery()
{
	// if landed (e and a)
	if ( this->flight_mode_ == FlightMode::LANDED ) {
		// limit loading to 100% battery capcaity
		if ( this->battery_ < 100.0 ) {
			this->battery_ += 1.0; //(1.0 / 15.0); // 1 percent per minute
		}

		// from e to a if battery > 80%
		if ( this->drone_mode_ == DroneMode::NOTREADY &&
			 this->battery_ > 80.0 ) {
			this->drone_mode_ = DroneMode::READY;
		}

	} else {
		this->battery_ -= 1.0; //(1.0 / 60.0); // (1 / 4) * (1 / 15);
	}

	// from c to d if battery < 20%
	if ( this->battery_ < 20.0 ) {
		this->drone_mode_ = DroneMode::NOTREADY;
		if ( this->flight_mode_ != FlightMode::LANDED ) {
			this->flight_mode_ = FlightMode::LANDING;
		}
	}

	RCLCPP_INFO(this->get_logger(), std::to_string(this->battery_).c_str());
}

void DroneController::flight()
{
	switch ( this->flight_mode_ ) {
	case FlightMode::LANDED:
		RCLCPP_INFO(this->get_logger(), "Landed");
		this->drone_landed();
		break;

	case FlightMode::STARTING:
		RCLCPP_INFO(this->get_logger(), "Starting");
		this->drone_start();
		this->publish_target();
		break;

	case FlightMode::FLYING:
		RCLCPP_INFO(this->get_logger(), "Flying");
		this->drone_flying();
		this->publish_target();
		break;

	case FlightMode::LANDING:
		RCLCPP_INFO(this->get_logger(), "Landing");
		this->drone_landing();
		this->publish_target();

		break;

	default:
		break;
	}
}

void DroneController::check_swarm_information()
{
	if ( this->drone_mode_ != DroneMode::NOTREADY ) {
		// if on a
		if ( this->drone_mode_ == DroneMode::READY &&
			 this->flight_mode_ == FlightMode::LANDED ) {
			if ( this->has_to_start() ) {
				// switch from a to b
				this->flight_mode_ = FlightMode::STARTING;
			}
		}
	}
}

bool DroneController::has_to_start() const
{
	const uint8_t num_drones_needed = 2;
	RCLCPP_INFO(this->get_logger(), "abcd");
	// check how many drones are in flying mode
	uint8_t num_drones_landed_ready = 0;
	uint8_t num_drones_flying_or_starting = 0;
	uint8_t num_drones_lower_id = 0;

	DroneMode drone_mode;
	FlightMode flight_mode;
	for ( const auto& drone : this->swarm_info_.swarm_positions.drones ) {

		drone_mode = static_cast<DroneMode>(drone.drone_mode);
		flight_mode = static_cast<FlightMode>(drone.flight_mode);

		if ( drone_mode == DroneMode::READY &&
			 flight_mode == FlightMode::LANDED ) {
			++num_drones_landed_ready;
			if ( drone.drone_id < this->id_ ) {
				++num_drones_lower_id;
			}
		}
		if ( drone_mode == DroneMode::FLYING ||
			 flight_mode == FlightMode::STARTING ) {
			++num_drones_flying_or_starting;
		}
	}

	const auto num_new_drones =
		num_drones_needed - num_drones_flying_or_starting;

	RCLCPP_INFO(
		this->get_logger(),
		std::string("landed ready: " + std::to_string(num_drones_landed_ready))
			.c_str());
	RCLCPP_INFO(this->get_logger(),
				std::string("flying/starting: " +
							std::to_string(num_drones_flying_or_starting))
					.c_str());

	RCLCPP_INFO(
		this->get_logger(),
		std::string("new drones: " + std::to_string(num_new_drones)).c_str());

	RCLCPP_INFO(this->get_logger(),
				std::string("lower id: " + std::to_string(num_drones_lower_id))
					.c_str());

	if ( num_new_drones > 0 ) {
		if ( num_drones_lower_id < num_new_drones ) {
			return true;
		} else {
			return false;
		}
	} else {
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

void DroneController::drone_flying()
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
	if ( dist < 0.05 ) {
		this->flight_mode_ = FlightMode::LANDED;
	} else {
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
