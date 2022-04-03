#include "sd_flight_controller/sd_flight_controller.hpp"

using namespace std::chrono_literals;
namespace sd {
namespace ros {
FlightController::FlightController() : Node("default_node_name")
{
	// node params
	// id of flight_controller
	this->id_ = this->get_name();
	// pid

	// pos subscriber
	this->pos_sub_ =
		this->create_subscription<sd_interfaces::msg::Position3Stamped>(
			"pos", 1,
			std::bind(&FlightController::on_position_msg_callback, this,
					  std::placeholders::_1));

	// pid timer
	this->flight_controller_timer_ = this->create_wall_timer(
		100ms,
		std::bind(&FlightController::flight_controller_timer_callback, this));

	// target_pos publisher
	this->target_pos_pub_ =
		this->create_publisher<sd_interfaces::msg::Position3Stamped>(
			"target_pos", 1);

	this->target_pos_.x = 10;
	this->target_pos_.y = 10;
	this->target_pos_.z = 10;

	// INFO
	RCLCPP_INFO(this->get_logger(),
				std::string("SD Flight Controller Node for Drone ID: " +
							this->id_ + " !")
					.c_str());
}

// timer callback for flight controller loop
void FlightController::flight_controller_timer_callback()
{
	this->publish_target_position();
}

// callback for each received pose msg
void FlightController::on_position_msg_callback(
	const sd_interfaces::msg::Position3Stamped::SharedPtr msg)
{
	// simply update last received msg
	this->last_pos_ = msg->pos3;
}

void FlightController::publish_target_position() const
{
	sd_interfaces::msg::Position3Stamped msg;
	// header
	msg.header.frame_id = this->get_name();
	msg.header.stamp = this->now();
	// pos3
	msg.pos3.set__x(this->target_pos_.x);
	msg.pos3.set__y(this->target_pos_.y);
	msg.pos3.set__z(this->target_pos_.z);

	this->target_pos_pub_->publish(msg);
}

void FlightController::set_target_position(
	const sd_interfaces::msg::Position3Stamped::SharedPtr pos)
{
	this->target_pos_ = pos->pos3;
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