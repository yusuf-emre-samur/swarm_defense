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
#include <sd_interfaces/msg/position_stamped.hpp>
#include <sd_interfaces/msg/swarm_info.hpp>
#include <sd_interfaces/srv/set_drone_target.hpp>
// other
#include <eigen3/Eigen/Dense>
#include <sd_drone_controller/sd_drone_controller.hpp>

namespace sd {

class FlightController : public rclcpp::Node
{
  public:
	// constructor
	FlightController();

  private:
	// callback function called each 0.5s
	void timer_callback();

	// set target
	void set_target();

	// check collisions
	void check_collision();

	// subscriber callbacks

	// callback to sub target
	void callback_target(const sd_interfaces::msg::FlightTarget& msg);

	// callback function to subscriber comm. receive
	void callback_comm_receive(const sd_interfaces::msg::SwarmInfo& msg);

	// callback function to subscriber to drone position
	void
	callback_position(const sd_interfaces::msg::PositionStamped::SharedPtr msg);

	// ros

	// callback timer
	rclcpp::TimerBase::SharedPtr timer_;

	// ros subscriber

	// subscriber to drone target
	rclcpp::Subscription<sd_interfaces::msg::FlightTarget>::SharedPtr
		sub_target_;

	// subscriber to comm. receive
	rclcpp::Subscription<sd_interfaces::msg::SwarmInfo>::SharedPtr
		sub_comm_receive_;

	// subscriber to drone pos
	rclcpp::Subscription<sd_interfaces::msg::PositionStamped>::SharedPtr
		sub_pos_;

	rclcpp::Client<sd_interfaces::srv::SetDroneTarget>::SharedPtr
		client_set_target_;

	// id of drone, e.g. 1
	uint8_t drone_id_;

	// name of drone, e.g. drone_1
	std::string name_;

	// current position of drone
	Eigen::Vector3d position_;

	// target position of drone
	Eigen::Vector3d target_;

	// if motors of drone are on
	bool motors_on_ = false;

	// target set flag
	bool target_set_ = false;

	std::mutex target_set_m;

	// positions of drones in swarm
	sd_interfaces::msg::SwarmInfo swarm_info_;
};

} // namespace sd

#endif