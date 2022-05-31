#ifndef SD_COMMUNICATION_HPP_
#define SD_COMMUNICATION_HPP_
// cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// rclcpp
#include <rclcpp/rclcpp.hpp>

// interfaces
#include <sd_interfaces/msg/drone_msg.hpp>
#include <sd_interfaces/msg/position.hpp>
#include <sd_interfaces/msg/world_objects.hpp>

namespace sd {

class DroneCommunication : public rclcpp::Node
{
  public:
	DroneCommunication();

  private:
	// functions
	void timer_callback();

	// parameters
	// ros
	rclcpp::TimerBase ::SharedPtr timer_;
	// rclcpp::Publisher<sd_interfaces::msg::Position>::SharedPtr publisher_;

	// drone
	int id_;
	std::string name_;
};

} // namespace sd

#endif