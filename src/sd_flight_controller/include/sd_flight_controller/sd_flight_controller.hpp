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
// other
#include <sd_flight_controller/pid.hpp>

namespace sd {
namespace ros {
class FlightController : public rclcpp::Node
{
  public:
	FlightController();

  private:
	/// functions
	// callbacks
	void on_pose_msg_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

	// timer callbacks
	void pid_timer_callback();

	/// vars
	rclcpp::Time last_time_;
	// pose
	std::string pose_sub_topic_name_; // name of pose topic
	geometry_msgs::msg::PoseStamped::SharedPtr last_pose_ =
		nullptr; // last pose msg
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
		pose_sub_; // pose subscriber

	// pid
	rclcpp::TimerBase::SharedPtr pid_timer_; // timer for pid

	// rpm
	std::string rpm_pub_topic_name_; // name of pose topic
	rclcpp::Publisher<sd_interfaces::msg::QuadcopterRPM>::SharedPtr
		rmp_pub_; // rpm publisher

	// pids
	std::unique_ptr<PID> pid_z;
	int rpm_thrust;
};
} // namespace ros
} // namespace sd

#endif