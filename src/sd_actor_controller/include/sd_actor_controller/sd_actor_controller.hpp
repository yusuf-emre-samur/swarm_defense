#ifndef SD_ACTOR_CONTROLLER_HPP_
#define SD_ACTOR_CONTROLLER_HPP_
// rcl
#include <rclcpp/rclcpp.hpp>
// interfaces
#include <sd_interfaces/msg/position3_stamped.hpp>
#include <std_msgs/msg/header.hpp>
// std
#include <chrono>
#include <functional>
#include <memory>
#include <string>
// other

namespace sd {
namespace ros {
class ActorController : public rclcpp::Node
{
  public:
	ActorController();

  private:
	// timer callback, flight controller loop
	void flight_controller_timer_callback();

	// pose sub callback
	void on_position_msg_callback(
		const sd_interfaces::msg::Position3Stamped::SharedPtr msg);

	// set goal pos
	void set_goal_position(
		const sd_interfaces::msg::Position3Stamped::SharedPtr pos);

	// publishs goal_pos_ msg to pose_pub_
	void publish_goal_position() const;

	// curr pose sub
	std::string pos_sub_topic_name_; // name of pose topic
	rclcpp::Subscription<sd_interfaces::msg::Position3Stamped>::SharedPtr
		pos_sub_; // pose subscriber

	// ros
	rclcpp::Time last_time_;

	// goal pose pub
	sd_interfaces::msg::Position3 last_pos_; // last pose msg
	std::string pos_pub_topic_name_;		 // name of pose topic
	rclcpp::Publisher<sd_interfaces::msg::Position3Stamped>::SharedPtr
		pos_pub_; // rpm publisher

	// fc
	rclcpp::TimerBase::SharedPtr flight_controller_timer_; // timer for fc

	// var
	sd_interfaces::msg::Position3 target_pos_;
};
} // namespace ros
} // namespace sd

#endif