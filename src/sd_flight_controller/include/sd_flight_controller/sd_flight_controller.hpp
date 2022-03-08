#ifndef SD_FLIGHT_CONTROLLER_HPP_
#define SD_FLIGHT_CONTROLLER_HPP_
// rcl
#include <rclcpp/rclcpp.hpp>
// interfaces
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sd_interfaces/msg/quadcopter_rpm.hpp>
// std
#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace sd {
namespace ros {
class FlightController : public rclcpp::Node
{
  public:
	FlightController();

  private:
	// callbacks
	void on_pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

	// timer callbacks
	void pid_timer_callback();

	// pose
	std::string drone_pose_topic_name_; // name of pose topic
	geometry_msgs::msg::PoseStamped::SharedPtr last_pose_ =
		nullptr; // last pose msg
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
		pose_subscriber_; // pose subscriber

	// pid
	rclcpp::TimerBase::SharedPtr pid_timer_; // timer for pid

	// rpm
	rclcpp::Publisher<sd_interfaces::msg::QuadcopterRPM>::SharedPtr
		rmp_pub_; // rpm publisher
};
} // namespace ros
} // namespace sd

#endif