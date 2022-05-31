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

	void update_knowledge();
	void sent_messages();

	// parameters
	// ros
	rclcpp::TimerBase ::SharedPtr timer_;

	// sub communication send
	rclcpp::Subscription<sd_interfaces::msg::DroneMsg>::SharedPtr
		sub_comm_send_;
	void callback_comm_send(const sd_interfaces::msg::DroneMsg& msg);

	// publisher communication receive
	rclcpp::Publisher<sd_interfaces::msg::DroneMsg>::SharedPtr
		pub_comm_receive_;

	// sub communication send
	rclcpp::Subscription<sd_interfaces::msg::DroneMsg>::SharedPtr
		sub_comm_incoming_;
	void callback_comm_incoming(const sd_interfaces::msg::DroneMsg& msg);

	// publisher communication receive
	rclcpp::Publisher<sd_interfaces::msg::DroneMsg>::SharedPtr
		pub_comm_outgoing_;

	// drone
	int id_;
	std::string name_;
};

} // namespace sd

#endif