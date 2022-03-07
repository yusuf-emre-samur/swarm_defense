#ifndef SD_FLIGHT_CONTROLLER_HPP_
#define SD_FLIGHT_CONTROLLER_HPP_
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace sd {
class SDFlightController : public rclcpp::Node
{
  public:
	SDFlightController();

  private:
	// callbacks
	void on_pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

	// pose subscriber
	std::string drone_pose_topic_name_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
		pose_subscriber_;
};

} // namespace sd

#endif