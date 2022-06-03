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

enum class DroneMode { NOTREADY = 0, READY = 1, FLYING = 2, MAX = 3 };

enum class FlightMode {
	LANDED = 0,
	STARTING = 1,
	LANDING = 2,
	FLYING = 3,
	RETURNING_HOME = 4,
	MAX = 5
};

class DroneController : public rclcpp::Node
{
  public:
	DroneController();

  private:
	// callback function of timer, called each 0.5s
	void timer_callback();

	void detect_threats();
	void filter_detected_threats();
	void calculate_pso_velocity();
	void publish_target();
	void publish_target(const Eigen::Vector3d& pos);
	void flight_to_base_station();
	void send_message_to_swarm();
	void check_swarm_information();
	void drone_start();
	void drone_flying();
	void drone_landing();
	void drone_landed();
	void set_drone_mode();
	void flight();

	void simulate_battery();

	bool has_to_start() const;

	void check_drone_landed();

	void turn_motors_off();

	// callback function for subcsriber to world objects list
	void callback_world_objects(
		const sd_interfaces::msg::WorldObjects::SharedPtr msg);

	// callback for subscriber to receiving communication
	void callback_comm_receive(const sd_interfaces::msg::SwarmInfo& msg);

	// callback function for subscriber to drone position
	void
	callback_position(const sd_interfaces::msg::PositionStamped::SharedPtr msg);

	// ros

	// timer fo callback
	rclcpp::TimerBase ::SharedPtr timer_;

	// ros subscriber

	// subscriber to world objects
	rclcpp::Subscription<sd_interfaces::msg::WorldObjects>::SharedPtr
		sub_world_objects_;

	// subscriber to communication receive
	rclcpp::Subscription<sd_interfaces::msg::SwarmInfo>::SharedPtr
		sub_comm_receive_;

	// subscriber to position
	rclcpp::Subscription<sd_interfaces::msg::PositionStamped>::SharedPtr
		sub_position_;

	// publisher to drone target
	rclcpp::Publisher<sd_interfaces::msg::FlightTarget>::SharedPtr pub_target_;

	// publisher to communication send
	rclcpp::Publisher<sd_interfaces::msg::DroneMsgOut>::SharedPtr
		pub_comm_send_;

	// id of drone, e.g. 1
	uint8_t id_;

	// name of drone, e.g. drone_1
	std::string name_;

	// current drone mode
	DroneMode drone_mode_ = DroneMode::READY;

	// current flight mode
	FlightMode flight_mode_ = FlightMode::LANDED;

	// current position of drone
	Eigen::Vector3d position_;

	// target of drone
	Eigen::Vector3d target_;

	// position of base station of drone
	Eigen::Vector3d base_station_pos_;

	// received information about swarm
	sd_interfaces::msg::SwarmInfo swarm_info_;

	// percentage of drone battery
	double battery_;

	// minimum number of drones which have to fly durin sim
	uint8_t min_flying_drones_;
};

} // namespace sd

#endif