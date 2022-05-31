#ifndef SD_DRONE_CONTROLLER_HPP_
#define SD_DRONE_CONTROLLER_HPP_
// cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// rclcpp
#include <rclcpp/rclcpp.hpp>

// interfaces
#include <sd_interfaces/msg/drone_msg.hpp>
#include <sd_interfaces/msg/position_stamped.hpp>
#include <sd_interfaces/msg/world_objects.hpp>

namespace sd {

enum DroneMode { FLYING, READY, NOTREADY, MAX };

class DroneController : public rclcpp::Node
{
  public:
	DroneController();

  private:
	// functions
	void timer_callback();

	void detect_threats();
	void filter_detected_threats();
	void calculate_pso_velocity();
	void set_target();
	void send_message_to_swarm();

	// ros subscriber
	// sub world objects
	rclcpp::Subscription<sd_interfaces::msg::WorldObjects>::SharedPtr
		sub_world_objects_;
	void callback_world_objects(const sd_interfaces::msg::WorldObjects& msg);

	// sub communication receive
	rclcpp::Subscription<sd_interfaces::msg::DroneMsg>::SharedPtr
		sub_comm_receive_;
	void callback_comm_receive(const sd_interfaces::msg::DroneMsg& msg);

	// sub position
	rclcpp::Subscription<sd_interfaces::msg::PositionStamped>::SharedPtr
		sub_position_;
	void callback_position(const sd_interfaces::msg::PositionStamped& msg);

	// ros publisher
	rclcpp::Publisher<sd_interfaces::msg::Position>::SharedPtr pub_target_;
	rclcpp::Publisher<sd_interfaces::msg::DroneMsg>::SharedPtr pub_comm_send_;

	// parameters
	// ros
	rclcpp::TimerBase ::SharedPtr timer_;
	// rclcpp::Publisher<sd_interfaces::msg::Position>::SharedPtr publisher_;

	// drone
	int id_;
	std::string name_;
	DroneMode drone_mode_ = DroneMode::READY;
};

} // namespace sd

#endif