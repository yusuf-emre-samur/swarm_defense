#include "sd_communication/sd_communication.hpp"

namespace sd {
DroneCommunication::DroneCommunication() : rclcpp::Node("DroneCommunication")
{
	// parameters
	// drone id
	this->declare_parameter<int>("drone_id", 0);
	this->get_parameter("drone_id", this->id_);
	this->name_ = "drone_" + std::to_string(this->id_);

	// timer
	using namespace std::chrono_literals;
	this->timer_ = rclcpp::create_timer(
		this, this->get_clock(), 500ms,
		std::bind(&DroneCommunication::timer_callback, this));

	// subscriber send
	this->sub_comm_send_ =
		this->create_subscription<sd_interfaces::msg::DroneMsg>(
			"communication_send", 1,
			std::bind(&DroneCommunication::callback_comm_send, this,
					  std::placeholders::_1));

	// publisher receive
	this->pub_comm_receive_ =
		this->create_publisher<sd_interfaces::msg::DroneMsg>(
			"communication_receive", 1);

	// subscriber incoming
	this->sub_comm_incoming_ =
		this->create_subscription<sd_interfaces::msg::DroneMsg>(
			"/drones/infos", 1,
			std::bind(&DroneCommunication::callback_comm_incoming, this,
					  std::placeholders::_1));

	// publisher outgoing
	this->pub_comm_outgoing_ =
		this->create_publisher<sd_interfaces::msg::DroneMsg>("/drones/infos",
															 1);
}

void DroneCommunication::timer_callback()
{
	this->sent_messages();
}

void DroneCommunication::callback_comm_send(
	const sd_interfaces::msg::DroneMsg& msg)
{
}

void DroneCommunication::callback_comm_incoming(
	const sd_interfaces::msg::DroneMsg& msg)
{
}

void DroneCommunication::sent_messages()
{
}

void DroneCommunication::update_knowledge()
{
}

} // namespace sd

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sd::DroneCommunication>());
	rclcpp::shutdown();
	return 0;
}
