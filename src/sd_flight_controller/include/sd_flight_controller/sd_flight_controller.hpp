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
#include <sd_interfaces/msg/drone_msgs.hpp>
#include <sd_interfaces/msg/flight_target.hpp>
#include <sd_interfaces/msg/position_stamped.hpp>
#include <sd_interfaces/msg/swarm_info.hpp>
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

	// check collisions
	void check_collision();

	// parameters
	// ros
	rclcpp::TimerBase ::SharedPtr timer_;

	// subscriber target
	rclcpp::Subscription<sd_interfaces::msg::FlightTarget>::SharedPtr
		sub_target_;
	void callback_target(const sd_interfaces::msg::FlightTarget& msg);

	// subscriber comm. receive
	rclcpp::Subscription<sd_interfaces::msg::SwarmInfo>::SharedPtr
		sub_comm_receive_;
	void
	callback_comm_receive(const sd_interfaces::msg::SwarmInfo::SharedPtr msg);

	// subscriber drone pos
	rclcpp::Subscription<sd_interfaces::msg::PositionStamped>::SharedPtr
		sub_pos_;
	void
	callback_position(const sd_interfaces::msg::PositionStamped::SharedPtr msg);

	rclcpp::Client<sd_interfaces::srv::SetDroneTarget>::SharedPtr
		client_set_target_;

	// drone
	uint8_t drone_id_;
	std::string name_;

	// position
	Eigen::Vector3d position_;
	Eigen::Vector3d target_;
	bool motors_on_ = false;
	std::mutex target_set_m;
	bool target_set_ = false;

	// sd_interfaces::msg::DroneMsgs drone_msgs_;
};

} // namespace sd

#endif