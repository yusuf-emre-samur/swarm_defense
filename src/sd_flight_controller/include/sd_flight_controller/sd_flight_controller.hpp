#ifndef SD_FLIGHT_CONTROLLER_HPP_
#define SD_FLIGHT_CONTROLLER_HPP_
// cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// rclcpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// interfaces
#include <sd_interfaces/msg/flight_target.hpp>
#include <sd_interfaces/msg/position.hpp>
#include <sd_interfaces/srv/set_drone_target.hpp>

// other
#include <eigen3/Eigen/Dense>

namespace sd {

class FlightController : public rclcpp::Node
{
  public:
	FlightController();

  private:
	// functions
	void timer_callback();

	// set target
	void set_target();

	// parameters
	// ros
	rclcpp::TimerBase ::SharedPtr timer_;

	rclcpp::Subscription<sd_interfaces::msg::FlightTarget>::SharedPtr
		sub_target_;
	void callback_target(const sd_interfaces::msg::FlightTarget& msg);

	rclcpp::Client<sd_interfaces::srv::SetDroneTarget>::SharedPtr
		client_set_target_;

	// drone
	int id_;
	std::string name_;

	// position
	Eigen::Vector3d target_;
	bool motors_on_ = false;
	std::mutex target_set_m;
	bool target_set_ = false;
};

} // namespace sd

#endif