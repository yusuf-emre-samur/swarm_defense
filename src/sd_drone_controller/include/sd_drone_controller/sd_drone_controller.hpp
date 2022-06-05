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

// drone mode
enum class DroneMode { NOTREADY = 0, READY = 1, FLYING = 2, MAX = 3 };

// flight mode of drone
enum class FlightMode {
	LANDED = 0,
	STARTING = 1,
	LANDING = 2,
	FLYING = 3,
	MAX = 4
};

class DroneController : public rclcpp::Node
{
  public:
	DroneController();

  private:
	// callback function of timer, called each 0.5s
	void timer_callback();

	//
	// main loop functions

	// share the knowledge of the drone to the swarm
	void share_knowledge_to_swarm();

	// process the information received from other swarms
	void process_swarm_information();

	// simulate the battery capacity and usage of the drone
	void simulate_battery();

	// process information with si algorithms and set new target
	void si_algorithms();

	// threat detection
	void detect_threats();

	//
	// PSO functions

	// calculate PSO velocity
	void calculate_pso_velocity();

	// calculate PSO fitness function
	void calculate_pso_fitness();

	// checks new positions score and set to pg if score is better
	void pso_update_pg(const double& score, const Eigen::Vector3d& pos);

	// checks new positions score and set to pb if score is better
	void pso_update_pb();

	// publish current target to flight controller
	void publish_target();

	// publish target to flight controller
	void publish_target(const Eigen::Vector3d& pos);

	// publish position of base station to flight controller
	void flight_to_base_station();

	//
	// functions for drone flying logic

	// flight logic of drone
	void flight();

	// called when drone is landed
	void drone_landed();

	// called when drone is starting
	void drone_starting();

	// called when drone is flying
	void drone_flying();

	// called when drone is landing
	void drone_landing();

	// checks if drone has to start
	void check_start();

	// helper function to check if target position is in bounding box
	bool in_bbox();

	// helper function to check if position is in bounding box
	bool in_bbox(const Eigen::Vector3d& pos);

	//
	// ROS subscriber callback functions

	// callback function for subcsriber to world objects list
	void callback_world_objects(const sd_interfaces::msg::WorldObjects& msg);

	// callback for subscriber to receiving communication
	void callback_comm_receive(const sd_interfaces::msg::SwarmInfo& msg);

	// callback function for subscriber to drone position
	void
	callback_position(const sd_interfaces::msg::PositionStamped::SharedPtr msg);

	//
	// Class members

	// ROS timer for callback function
	rclcpp::TimerBase ::SharedPtr timer_;

	//
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

	//
	// ROS publisher

	// publisher to drone target
	rclcpp::Publisher<sd_interfaces::msg::FlightTarget>::SharedPtr pub_target_;

	// publisher to communication send
	rclcpp::Publisher<sd_interfaces::msg::DroneMsgOut>::SharedPtr
		pub_comm_send_;

	//
	// Drone Information

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

	// list of world objetcs
	sd_interfaces::msg::WorldObjects world_objects_;

	// radius where drone can detect objects
	double perception_radius_ = 10.0;

	// percentage of drone battery
	double battery_;

	// minimum number of drones which have to fly durin sim
	uint8_t min_flying_drones_;

	// used for drone to check if it has to start
	uint start_counter_ = 0;

	// operation height of drone
	double drone_op_height_;

	//
	// PSO parameters

	// PSO coefficients
	double w_;
	double c1_;
	double c2_;

	// current score
	double pso_score_;

	// personal best position
	Eigen::Vector3d pb_;

	// personal best score
	double pb_score_;

	// swarm best position
	Eigen::Vector3d pg_;

	// swarm best score
	double pg_score_;

	// current velocity of drone
	Eigen::Vector3d velocity_;

	// max velocity of drone
	Eigen::Array3d max_velocity_;

	//
	// threats
	std::vector<sd_interfaces::msg::Threat> swarm_threats_;

	bool following_threat_ = false;

	std::vector<sd_interfaces::msg::Position> ignore_regions_;
	double ignore_radius_ = 10.0;

	Eigen::Vector2d bbox_max_;
	Eigen::Vector2d bbox_min_;
};

} // namespace sd

#endif