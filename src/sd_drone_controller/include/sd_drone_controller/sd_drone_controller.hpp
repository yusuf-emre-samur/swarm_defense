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
#include <sd_interfaces/msg/drone_msg_out.hpp>
#include <sd_interfaces/msg/drone_msgs.hpp>
#include <sd_interfaces/msg/flight_target.hpp>
#include <sd_interfaces/msg/position_stamped.hpp>
#include <sd_interfaces/msg/swarm_info.hpp>
#include <sd_interfaces/msg/world_objects.hpp>

// other
#include <eigen3/Eigen/Dense>

namespace sd {

enum class DroneMode { FLYING = 0, READY = 1, NOTREADY = 2, MAX = 3 };

enum class FlightMode {
	LANDED = 0,
	STARTING = 1,
	LADNING = 2,
	FLYING = 3,
	RETURNING_HOME = 4,
	MAX = 5
};

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
	void set_target(const Eigen::Vector3d& pos);
	void flight_to_base_station();
	void send_message_to_swarm();

	// helper

	// ros subscriber
	// sub world objects
	rclcpp::Subscription<sd_interfaces::msg::WorldObjects>::SharedPtr
		sub_world_objects_;
	void callback_world_objects(
		const sd_interfaces::msg::WorldObjects::SharedPtr msg);

	// sub communication receive
	rclcpp::Subscription<sd_interfaces::msg::SwarmInfo>::SharedPtr
		sub_comm_receive_;
	void
	callback_comm_receive(const sd_interfaces::msg::SwarmInfo::SharedPtr msg);

	// sub position
	rclcpp::Subscription<sd_interfaces::msg::PositionStamped>::SharedPtr
		sub_position_;
	void
	callback_position(const sd_interfaces::msg::PositionStamped::SharedPtr msg);

	// ros publisher
	rclcpp::Publisher<sd_interfaces::msg::FlightTarget>::SharedPtr pub_target_;
	rclcpp::Publisher<sd_interfaces::msg::DroneMsgOut>::SharedPtr
		pub_comm_send_;

	// parameters
	// ros
	rclcpp::TimerBase ::SharedPtr timer_;
	// rclcpp::Publisher<sd_interfaces::msg::Position>::SharedPtr publisher_;

	// drone
	uint8_t id_;
	std::string name_;
	DroneMode drone_mode_ = DroneMode::READY;
	FlightMode flight_mode_ = FlightMode::LANDED;

	Eigen::Vector3d position_;
	Eigen::Vector3d target_;
	Eigen::Vector3d base_station_pos_;
};

} // namespace sd

#endif