#include "sd_drone_controller/sd_drone_controller.hpp"

namespace sd {
DroneController::DroneController() : rclcpp::Node("DroneController")
{
	// parameters
	// drone id
	this->declare_parameter<int>("drone_id", 0);
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

	// this->base_station_pos_

	using namespace std::chrono_literals;
	this->timer_ =
		rclcpp::create_timer(this, this->get_clock(), 500ms,
							 std::bind(&DroneController::timer_callback, this));

	// ros subscriber
	this->sub_world_objects_ =
		this->create_subscription<sd_interfaces::msg::WorldObjects>(
			"/world/objects", 1,
			std::bind(&DroneController::callback_world_objects, this,
					  std::placeholders::_1));

	this->sub_comm_receive_ =
		this->create_subscription<sd_interfaces::msg::DroneMsg>(
			"communication_receive", 1,
			std::bind(&DroneController::callback_comm_receive, this,
					  std::placeholders::_1));

	this->sub_position_ =
		this->create_subscription<sd_interfaces::msg::PositionStamped>(
			"position", 1,
			std::bind(&DroneController::callback_position, this,
					  std::placeholders::_1));

	// ros publisher
	this->pub_target_ =
		this->create_publisher<sd_interfaces::msg::FlightTarget>("target", 1);
}

void DroneController::timer_callback()
{
	this->detect_threats();
	this->filter_detected_threats();
	this->calculate_pso_velocity();
	if ( this->flight_mode_ == FlightMode::FLYING ) {
		this->set_target();
	}

	this->send_message_to_swarm();
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

void DroneController::set_target()
{
	this->set_target(this->position_);
}

void DroneController::set_target(const Eigen::Vector3d& pos)
{
	sd_interfaces::msg::FlightTarget target;
	target.pos.x = pos.x();
	target.pos.y = pos.y();
	target.pos.z = pos.z();

	this->pub_target_->publish(target);
}

void DroneController::flight_to_base_station()
{
	this->set_target(this->base_station_pos_);
}

void DroneController::send_message_to_swarm()
{
}

// subscirber world objects
void DroneController::callback_world_objects(
	const sd_interfaces::msg::WorldObjects& msg)
{
}

// subscriber communincation receive
void DroneController::callback_comm_receive(
	const sd_interfaces::msg::DroneMsg& msg)
{
}

// subscriber position
void DroneController::callback_position(
	const sd_interfaces::msg::PositionStamped& msg)
{
	this->position_.x() = msg.position.x;
	this->position_.y() = msg.position.y;
	this->position_.z() = msg.position.z;
}

} // namespace sd

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sd::DroneController>());
	rclcpp::shutdown();
	return 0;
}
