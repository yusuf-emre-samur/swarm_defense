#include "sd_flight_controller/sd_flight_controller.hpp"

using namespace std::chrono_literals;
namespace sd {
namespace ros {
FlightController::FlightController() : Node("default_node_name")
{
	// node params
	this->declare_parameter<std::string>("drone_pose_topic", "");
	this->get_parameter("drone_pose_topic", this->drone_pose_topic_name_);

	// pose subscriber
	this->pose_subscriber_ =
		this->create_subscription<geometry_msgs::msg::PoseStamped>(
			this->drone_pose_topic_name_, 1,
			std::bind(&FlightController::on_pose_callback, this,
					  std::placeholders::_1));

	// pid timer
	this->pid_timer_ = this->create_wall_timer(
		1s, std::bind(&FlightController::pid_timer_callback, this));
	RCLCPP_INFO(this->get_logger(), "SD Flight Controller Node started!");

	// rpm publisher
	this->rmp_pub_ =
		this->create_publisher<sd_interfaces::msg::QuadcopterRPM>("rpm", 10);
}

// callback for each received pose msg
void FlightController::on_pose_callback(
	geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	// simply update last received msg
	this->last_pose_ = msg;
}

// timer callback for pid controller
void FlightController::pid_timer_callback()
{
	// do pid

	// calc motor rpms

	sd_interfaces::msg::QuadcopterRPM msg;
	msg.rotor0.rpm = 0;
	msg.rotor1.rpm = 0;
	msg.rotor2.rpm = 0;
	msg.rotor3.rpm = 0;
	// publish rpm message
	this->rmp_pub_->publish(msg);
	RCLCPP_INFO(this->get_logger(),
				std::to_string(this->now().seconds()).c_str());
}

} // namespace ros
} // namespace sd

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sd::ros::FlightController>());
	rclcpp::shutdown();
	return 0;
}