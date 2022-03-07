#include "sd_flight_controller/sd_flight_controller.hpp"
#include <string>

namespace sd {

SDFlightController::SDFlightController() : Node("default_node_name")
{
	this->declare_parameter<std::string>("drone_pose_topic", "");
	this->get_parameter("drone_pose_topic", this->drone_pose_topic_name_);

	this->pose_subscriber_ =
		this->create_subscription<geometry_msgs::msg::PoseStamped>(
			this->drone_pose_topic_name_, 1,
			std::bind(&SDFlightController::on_pose_callback, this,
					  std::placeholders::_1));

	RCLCPP_INFO(this->get_logger(), "SD Flight Controller Node started!");
}

void SDFlightController::on_pose_callback(
	geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	RCLCPP_INFO(this->get_logger(),
				std::to_string(msg->pose.position.x).c_str());
}

} // namespace sd

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sd::SDFlightController>());
	rclcpp::shutdown();
	return 0;
}