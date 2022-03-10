#ifndef SD_FLIGHT_CONTROLLER_HPP_
#define SD_FLIGHT_CONTROLLER_HPP_
// rcl
#include <rclcpp/rclcpp.hpp>
// interfaces
#include <geometry_msgs/msg/pose_stamped.hpp>
// std
#include <chrono>
#include <functional>
#include <memory>
#include <string>
// other

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
	void on_pose_msg_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

	// set goal pos
	void set_goal_pos(geometry_msgs::msg::Pose::SharedPtr pos);

	// publishs goal_pos_ msg to pose_pub_
	void publish_goal_pos() const;

	// curr pose sub
	std::string pose_sub_topic_name_; // name of pose topic
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
		pose_sub_; // pose subscriber

	// ros
	rclcpp::Time last_time_;

	// goal pose pub
	geometry_msgs::msg::PoseStamped::SharedPtr last_pose_ =
		nullptr;					  // last pose msg
	std::string pose_pub_topic_name_; // name of pose topic
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
		pose_pub_; // rpm publisher

	// fc
	rclcpp::TimerBase::SharedPtr flight_controller_timer_; // timer for fc

	// var
	geometry_msgs::msg::Pose goal_pose_;
};
} // namespace ros
} // namespace sd

#endif