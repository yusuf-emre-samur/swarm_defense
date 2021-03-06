#ifndef SD_COMMUNICATION_HPP_
#define SD_COMMUNICATION_HPP_
// cpp
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
// rclcpp
#include <rclcpp/rclcpp.hpp>

// interfaces
#include <sd_interfaces/msg/drone_msg_out.hpp>
#include <sd_interfaces/msg/position.hpp>
#include <sd_interfaces/msg/swarm_info.hpp>
#include <sd_interfaces/msg/world_objects.hpp>
// other
#include <eigen3/Eigen/Dense>

namespace sd {

class DroneCommunication : public rclcpp::Node
{
  public:
	DroneCommunication();

  private:
	// functions
	void timer_callback();

	void update_knowledge();
	double old_after__seconds_;

	// sent msg from drone controller to drones
	void sent_messages_outgoing();

	// sent msg from drones to drone controller
	void sent_messages_incoming();
	// sd_interfaces::msg::DroneMsgs msg_incoming_;

	// parameters
	// ros
	rclcpp::TimerBase::SharedPtr timer_;

	// sub communication send
	rclcpp::Subscription<sd_interfaces::msg::DroneMsgOut>::SharedPtr
		sub_comm_send_;
	void callback_comm_send(const sd_interfaces::msg::DroneMsgOut& msg);

	// publisher communication receive
	rclcpp::Publisher<sd_interfaces::msg::SwarmInfo>::SharedPtr
		pub_comm_receive_;

	// sub communication incoming
	rclcpp::Subscription<sd_interfaces::msg::DroneMsgOut>::SharedPtr
		sub_comm_incoming_;
	void callback_comm_incoming(
		const sd_interfaces::msg::DroneMsgOut::SharedPtr msg);

	// publisher communication send out
	rclcpp::Publisher<sd_interfaces::msg::DroneMsgOut>::SharedPtr
		pub_comm_outgoing_;

	// drone
	uint8_t id_;
	std::string name_;

	sd_interfaces::msg::DroneMsgOut::SharedPtr msg_send_;

	std::mutex swarm_info_mutex_;
	sd_interfaces::msg::SwarmInfo swarm_info_;

	std::vector<sd_interfaces::msg::Threat> swarm_threats_;

	rclcpp::Time last_time_;
};

} // namespace sd

#endif