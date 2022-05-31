#include "sd_drone_controller/sd_drone_controller.hpp"

namespace sd {
DroneController::DroneController() : rclcpp::Node("DroneController")
{
	// parameters
	// drone id
	this->declare_parameter<int>("drone_id", 0);
	this->get_parameter("drone_id", this->id_);
	this->name_ = "drone_" + std::to_string(this->id_);

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
		this->create_publisher<sd_interfaces::msg::Position>("target", 1);
}

void DroneController::timer_callback()
{
	this->detect_threats();
	this->filter_detected_threats();
	this->calculate_pso_velocity();
	this->set_target();
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
}

} // namespace sd

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sd::DroneController>());
	rclcpp::shutdown();
	return 0;
}
