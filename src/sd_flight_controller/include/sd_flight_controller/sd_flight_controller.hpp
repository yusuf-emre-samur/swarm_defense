#ifndef SD_FLIGHT_CONTROLLER_HPP_
#define SD_FLIGHT_CONTROLLER_HPP_

// std
#include <chrono>
#include <functional>
#include <memory>
#include <string>
// rcl
#include <rclcpp/rclcpp.hpp>
// interfaces
#include <sd_interfaces/msg/position_stamped.hpp>
#include <std_msgs/msg/header.hpp>

namespace sd {
namespace ros {
class FlightController : public rclcpp::Node
{
  public:
	FlightController();

  private:
	// timer callback, flight controller loop
	void flight_controller_timer_callback();

	// pose sub callback
	void on_position_msg_callback(
		const sd_interfaces::msg::PositionStamped::SharedPtr msg);

	// set goal pos
	void set_target_position(
		const sd_interfaces::msg::PositionStamped::SharedPtr pos);

	// publishs goal_pos_ msg to pose_pub_
	void publish_target_position() const;

	std::string id_;

	// curr pos sub
	rclcpp::Subscription<sd_interfaces::msg::PositionStamped>::SharedPtr
		pos_sub_; // pose subscriber

	// last pos msg
	sd_interfaces::msg::Position last_pos_;
	// target pos pub
	rclcpp::Publisher<sd_interfaces::msg::PositionStamped>::SharedPtr
		target_pos_pub_;

	// fc
	rclcpp::TimerBase::SharedPtr flight_controller_timer_; // timer for fc

	// var
	sd_interfaces::msg::Position target_pos_;
};
} // namespace ros
} // namespace sd

#endif