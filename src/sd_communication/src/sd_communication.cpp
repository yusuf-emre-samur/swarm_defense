#include "sd_communication/sd_communication.hpp"

namespace sd {
DroneCommunication::DroneCommunication() : rclcpp::Node("DroneCommunication")
{
	// parameters
	// drone id
	this->declare_parameter<int>("drone_id", 0);
	this->get_parameter("drone_id", this->id_);
	this->name_ = "drone_" + std::to_string(this->id_);

	using namespace std::chrono_literals;
	this->timer_ = this->create_wall_timer(
		500ms, std::bind(&DroneCommunication::timer_callback, this));
}

void DroneCommunication::timer_callback()
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
