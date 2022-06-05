#include "sd_drone_controller/sd_drone_controller.hpp"
#include <sd_drone_controller/sd_utils.hpp>

#include <random>
#include <stdio.h>	/* printf, NULL */
#include <stdlib.h> /* srand, rand */
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

std::random_device rd;
std::mt19937 generator(rd());
std::uniform_real_distribution<double> distribution(0.0, 1.0);
std::uniform_real_distribution<double> distribution2(-50.0, 50.0);

namespace sd {
DroneController::DroneController() : rclcpp::Node("DroneController")
{
	// parameters
	// drone id
	this->declare_parameter<uint8_t>("drone_id", 0);
	this->get_parameter("drone_id", this->id_);
	this->name_ = "drone_" + std::to_string(this->id_);

	// drone base station
	this->declare_parameter<std::vector<double>>("base_station_pos");
	rclcpp::Parameter base_station_pos("base_station_pos",
									   std::vector<double>({}));
	this->get_parameter("base_station_pos", base_station_pos);
	auto tmp_double_vect = base_station_pos.as_double_array();
	this->base_station_pos_.x() = tmp_double_vect[0];
	this->base_station_pos_.y() = tmp_double_vect[1];
	this->base_station_pos_.z() = tmp_double_vect[2];

	// starting battery percentage
	this->declare_parameter<uint8_t>("min_flying_drones", 2);
	this->get_parameter("min_flying_drones", this->min_flying_drones_);

	// starting battery percentage
	this->declare_parameter<double>("battery", 100.0);
	this->get_parameter("battery", this->battery_);

	// pso parameters
	this->declare_parameter<double>("pso_w", 0.9);
	this->get_parameter("pso_c1", this->w_);

	this->declare_parameter<double>("pso_c1", 1.49);
	this->get_parameter("pso_c1", this->c1_);

	this->declare_parameter<double>("pso_c2", 1.49);
	this->get_parameter("pso_c2", this->c2_);

	// max velocity
	this->declare_parameter<std::vector<double>>("max_velocity");
	rclcpp::Parameter pso_max_vel("max_velocity",
								  std::vector<double>({5.0, 5.0, 0.0}));
	this->get_parameter("max_velocity", pso_max_vel);
	tmp_double_vect = pso_max_vel.as_double_array();
	this->max_velocity_.x() = tmp_double_vect[0];
	this->max_velocity_.y() = tmp_double_vect[1];
	this->max_velocity_.z() = tmp_double_vect[2];

	// operating bbox
	// bbox max
	this->declare_parameter<std::vector<double>>("bbox_max");
	rclcpp::Parameter bbox_max("bbox_max", std::vector<double>({}));
	this->get_parameter("bbox_max", bbox_max);
	tmp_double_vect = bbox_max.as_double_array();
	this->bbox_max_.x() = tmp_double_vect[0];
	this->bbox_max_.y() = tmp_double_vect[1];
	// bbox max
	this->declare_parameter<std::vector<double>>("bbox_min");
	rclcpp::Parameter bbox_min("bbox_min", std::vector<double>({}));
	this->get_parameter("bbox_min", bbox_min);
	tmp_double_vect = bbox_min.as_double_array();
	this->bbox_min_.x() = tmp_double_vect[0];
	this->bbox_min_.y() = tmp_double_vect[1];

	// set pb and pg to max
	this->pb_score_ = std::numeric_limits<double>::max();
	this->pg_score_ = std::numeric_limits<double>::max();
	this->pso_score_ = std::numeric_limits<double>::max();

	this->pg_ = 100 * Eigen::Vector3d::Random();
	this->pb_ = 100 * Eigen::Vector3d::Random();

	// set drone mode dependent of batter capacity
	if ( this->battery_ < 80.0 ) {
		// start at e
		this->drone_mode_ = DroneMode::NOTREADY;
	} else {
		// start at a
		this->drone_mode_ = DroneMode::READY;
	}
	// all drones are starting landend
	this->flight_mode_ = FlightMode::LANDED;

	// set unique operating height for each drone,  depending on id
	this->drone_op_height_ = static_cast<double>(this->id_) * 1.5 + 15;

	// main loop timer
	using namespace std::chrono_literals;
	this->timer_ =
		rclcpp::create_timer(this, this->get_clock(), 100ms,
							 std::bind(&DroneController::timer_callback, this));
	//
	// ROS subscriber

	// sub world objects
	this->sub_world_objects_ =
		this->create_subscription<sd_interfaces::msg::WorldObjects>(
			"/world/objects", 1,
			std::bind(&DroneController::callback_world_objects, this,
					  std::placeholders::_1));
	// comm. receive
	this->sub_comm_receive_ =
		this->create_subscription<sd_interfaces::msg::SwarmInfo>(
			"communication_receive", 1,
			std::bind(&DroneController::callback_comm_receive, this,
					  std::placeholders::_1));

	// sub drone position
	this->sub_position_ =
		this->create_subscription<sd_interfaces::msg::PositionStamped>(
			"position", 1,
			std::bind(&DroneController::callback_position, this,
					  std::placeholders::_1));

	//
	// ROS publisher

	// pub comm. send
	this->pub_comm_send_ = create_publisher<sd_interfaces::msg::DroneMsgOut>(
		"communication_send", 1);

	// pub drone target
	this->pub_target_ =
		this->create_publisher<sd_interfaces::msg::FlightTarget>("target", 1);
}

// main loop
void DroneController::timer_callback()
{
	this->share_knowledge_to_swarm();
	this->process_swarm_information();
	this->simulate_battery();
	this->detect_threats();
	this->flight();
}

void DroneController::share_knowledge_to_swarm()
{
	sd_interfaces::msg::DroneMsgOut msg;
	// header
	msg.drone_header.drone_id = this->id_;
	msg.drone_header.drone_mode = static_cast<uint8_t>(this->drone_mode_);
	msg.drone_header.flight_mode = static_cast<uint8_t>(this->flight_mode_);
	msg.drone_header.stamp = this->now();
	// pos
	msg.drone_header.pos.x = this->position_.x();
	msg.drone_header.pos.y = this->position_.y();
	msg.drone_header.pos.z = this->position_.z();
	// battery
	msg.drone_header.battery = this->battery_;

	// pso
	msg.pb = eigen_to_sd_pos(this->pb_);
	msg.pb_score = this->pb_score_;

	//  threats
	msg.detected_threats = this->swarm_threats_;

	this->pub_comm_send_->publish(msg);
}

void DroneController::process_swarm_information()
{
	if ( this->drone_mode_ != DroneMode::NOTREADY ) {
		this->check_start();
	}

	// create ignore regions
	for ( const auto& drone : this->swarm_info_.swarm_drones ) {
		auto dist = (this->position_ - sd_pos_to_eigen(drone.pos))
						.segment(0, 2)
						.cwiseAbs()
						.norm();
		if ( dist < this->ignore_radius_ ) {
			this->ignore_regions_.push_back(drone.pos);
		}
	}
}

void DroneController::simulate_battery()
{
	// if landed (e and a)
	if ( this->flight_mode_ == FlightMode::LANDED ) {
		// limit loading to 100% battery capcaity
		if ( this->battery_ < 100.0 ) {
			this->battery_ += (1.0 / 15.0); // 1 percent per minute
		} else {
			this->battery_ = 100.0;
		}

		// from e to a if battery > 80%
		if ( this->drone_mode_ == DroneMode::NOTREADY &&
			 this->battery_ > 80.0 ) {
			this->drone_mode_ = DroneMode::READY;
		}

	} else {
		this->battery_ -= (1.0 / 60.0);
		// (1.0 / 60.0); // (1 / 4) * (1 / 15); // 4 percent per minute
	}

	// from c to d if battery < 20%
	if ( this->battery_ < 20.0 ) {
		this->drone_mode_ = DroneMode::NOTREADY;
		if ( this->flight_mode_ != FlightMode::LANDED ) {
			this->flight_mode_ = FlightMode::LANDING;
		}
	}
}

void DroneController::flight()
{
	switch ( this->flight_mode_ ) {
	case FlightMode::LANDED:
		this->drone_landed();
		break;

	case FlightMode::STARTING:
		this->drone_starting();
		this->publish_target();
		break;

	case FlightMode::LANDING:
		this->drone_landing();
		this->publish_target();
		break;

	case FlightMode::FLYING:
		this->drone_flying();
		this->publish_target();
		break;

	default:
		break;
	}
}

void DroneController::si_algorithms()
{
	//
	// PSO algorithm
	//
	// update pg from received pg and score
	this->pso_update_pg(this->swarm_info_.pg_score,
						sd_pos_to_eigen(this->swarm_info_.pg));

	// calculate current fitness
	this->calculate_pso_fitness();

	// update pb score
	this->pso_update_pb();

	// calculate velocity
	this->calculate_pso_velocity();
}

void DroneController::calculate_pso_velocity()
{

	//
	// pso velocity
	auto r1 = distribution(generator);
	auto r2 = distribution(generator);

	this->velocity_ = (this->w_ * this->velocity_ +
					   r1 * this->c1_ * (this->pb_ - this->position_) +
					   r2 * this->c2_ * (this->pg_ - this->position_));

	this->velocity_ =
		this->velocity_.normalized().array() * this->max_velocity_;

	//
	// vector from other flying drones
	auto vector_out = Eigen::Vector3d();
	uint8_t num_flying_drones = 0;
	for ( const auto& drone : this->swarm_info_.swarm_drones ) {
		if ( static_cast<DroneMode>(drone.drone_mode) == DroneMode::FLYING ) {

			vector_out += (sd_pos_to_eigen(drone.pos) - this->position_);
			++num_flying_drones;
		}
	}
	vector_out /= static_cast<double>(num_flying_drones);

	if ( vector_out.normalized().cwiseAbs().norm() > 5.0 ) {
		this->velocity_ = vector_out.normalized().array() * this->max_velocity_;
	}
}

void DroneController::calculate_pso_fitness()
{
	double x_pos = this->position_.x();
	double y_pos = this->position_.y();

	double z_val = 0.0;

	// distance to detected threats
	for ( const auto& threat : this->swarm_threats_ ) {
		// positive feedback
		z_val += 0.01 * (std::pow((x_pos - threat.pos.x), 2) +
						 std::pow(y_pos - threat.pos.y, 2)) -
				 100;
	}

	// distance to flying drones
	for ( const auto& drone : this->swarm_info_.swarm_drones ) {
		if ( static_cast<DroneMode>(drone.drone_mode) == DroneMode::FLYING ) {
			// negative feedback
			z_val -= 0.01 * (std::pow((x_pos - drone.pos.x), 2) +
							 std::pow(y_pos - drone.pos.y, 2));
		}
	}

	// for ( const auto& threat : this->swarm_threats_ ) {
	// 	for ( const auto& drone : this->swarm_info_.swarm_drones ) {
	// 		if ( static_cast<DroneMode>(drone.drone_mode) ==
	// 			 DroneMode::FLYING ) {
	// 			auto dist =
	// 				(sd_pos_to_eigen(threat.pos) - sd_pos_to_eigen(drone.pos))
	// 					.segment(0, 2)
	// 					.cwiseAbs()
	// 					.norm();
	// 			if ( dist < this->perception_radius_ ) {
	// 				RCLCPP_INFO(this->get_logger(), "another drone in near");
	// 				// negative feedback
	// 				z_val -= 0.00001 * (std::pow((x_pos - threat.pos.x), 2) +
	// 									std::pow(y_pos - threat.pos.y, 2));
	// 			}
	// 		}
	// 	}
	// }

	//
	// check ignore regions:

	this->pso_score_ = z_val;

	RCLCPP_INFO(
		this->get_logger(),
		std::string("score: " + std::to_string(this->pso_score_)).c_str());
}

void DroneController::pso_update_pg(const double& score,
									const Eigen::Vector3d& pos)
{
	if ( score < this->pg_score_ ) {
		this->pg_score_ = score;
		this->pg_ = pos;
	}
}

void DroneController::pso_update_pb()
{
	if ( this->pso_score_ < this->pb_score_ ) {
		this->pb_score_ = this->pso_score_;
		this->pb_ = this->position_;
	}
}

void DroneController::detect_threats()
{
	// reset detected threats
	this->swarm_threats_.clear();

	// go through world objects
	for ( const auto& object : this->world_objects_.objects ) {
		sd_interfaces::msg::Threat t;
		t.pos = object.bbox.center;
		t.following_drones_id.push_back(this->id_);
		t.time_detected = this->now();

		auto t_pos = sd_pos_to_eigen(t.pos);
		auto abcd = (this->position_ - t_pos);
		auto dist = abcd.segment(0, 2).cwiseAbs().norm();

		bool threat_not_found = true;
		if ( dist < this->perception_radius_ ) {
			// check if threat already exists
			if ( this->swarm_threats_.size() > 0 ) {

				// iterate over existing threats
				for ( const auto& existing_threat : this->swarm_threats_ ) {

					// check distance between threats
					auto threats_dist =
						(sd_pos_to_eigen(existing_threat.pos) - t_pos)
							.segment(0, 2)
							.cwiseAbs()
							.norm();

					// if distance between threats is small, then threat is the
					// same
					if ( threats_dist < 2.0 ) {
						RCLCPP_INFO(this->get_logger(),
									std::string("detected threat: " + object.id)
										.c_str());
						RCLCPP_INFO(this->get_logger(),
									std::string("threat distance : " +
												std::to_string(dist))
										.c_str());

						RCLCPP_INFO(
							this->get_logger(),
							std::string("t dist x: " + std::to_string(abcd.x()))
								.c_str());
						RCLCPP_INFO(
							this->get_logger(),
							std::string("t dist y: " + std::to_string(abcd.y()))
								.c_str());

						threat_not_found = false;
						break;
					}
				}
				if ( threat_not_found ) {
					this->swarm_threats_.push_back(t);
				}
			} else {
				RCLCPP_INFO(
					this->get_logger(),
					std::string("detected threat: " + object.id).c_str());
				RCLCPP_INFO(
					this->get_logger(),
					std::string("threat distance : " + std::to_string(dist))
						.c_str());
				RCLCPP_INFO(this->get_logger(),
							std::string("t dist x: " + std::to_string(abcd.x()))
								.c_str());
				RCLCPP_INFO(this->get_logger(),
							std::string("t dist y: " + std::to_string(abcd.y()))
								.c_str());
				this->swarm_threats_.push_back(t);
			}
		}
	}
}

void DroneController::check_start()
{
	const int8_t num_drones_needed = this->min_flying_drones_;
	// check how many drones are in flying mode
	int8_t num_drones_landed_ready = 0;
	int8_t num_drones_flying_or_starting = 0;
	int8_t num_drones_lower_id = 0;

	DroneMode drone_mode;
	FlightMode flight_mode;
	for ( const auto& drone : this->swarm_info_.swarm_drones ) {

		drone_mode = static_cast<DroneMode>(drone.drone_mode);
		flight_mode = static_cast<FlightMode>(drone.flight_mode);

		if ( drone_mode != DroneMode::NOTREADY ) {
			// not ready
			if ( (flight_mode == FlightMode::LANDED) &&
				 (drone_mode == DroneMode::READY) ) {
				// ready and landed
				num_drones_landed_ready++;
			}

			if ( (flight_mode == FlightMode::FLYING) ||
				 (flight_mode == FlightMode::STARTING) ) {

				++num_drones_flying_or_starting;
			}
			if ( drone_mode == DroneMode::READY ) { //
				if ( drone.drone_id < this->id_ ) {
					++num_drones_lower_id;
				}
			}
		}

		// if ( num_drones_flying_or_starting >= num_drones_needed ) {
		// 	return false;
		// }
	}

	if ( (this->flight_mode_ == FlightMode::FLYING) ||
		 (this->flight_mode_ == FlightMode::STARTING) ) {
		++num_drones_flying_or_starting;
	}
	if ( (this->drone_mode_ == DroneMode::READY) ) {
		++num_drones_landed_ready;
	}

	const int8_t num_new_drones =
		num_drones_needed - num_drones_flying_or_starting;

	const int8_t num_too_much_drones =
		num_drones_flying_or_starting - num_drones_needed;

	if ( num_new_drones > 0 ) {
		// new drones needed
		if ( num_drones_lower_id <= num_new_drones ) { //
			this->flight_mode_ = FlightMode::STARTING;
		}
	} else if ( num_too_much_drones > 0 ) {
		// too much drones
		if ( num_too_much_drones <= num_drones_lower_id ) {
			// have to land
			if ( this->flight_mode_ == FlightMode::STARTING ) {
				this->flight_mode_ = FlightMode::LANDING;
			}
		}
	}
}

void DroneController::drone_landed()
{
	// turn motors off
	sd_interfaces::msg::FlightTarget target;
	target.motors_on = false;
	this->pub_target_->publish(target);
	this->start_counter_ = 0;
}

void DroneController::drone_starting()
{
	auto target =
		this->base_station_pos_ + Eigen::Vector3d(0, 0, this->drone_op_height_);
	auto dist = (this->position_ - target).cwiseAbs().norm();
	if ( dist < 0.05 ) {
		this->flight_mode_ = FlightMode::FLYING;
		this->drone_mode_ = DroneMode::FLYING;
		this->start_counter_ = 0;
	} else {
		++this->start_counter_;
		if ( this->start_counter_ > 10 ) {
			this->target_ = target;
		}
	}
}

void DroneController::drone_flying()
{
	this->si_algorithms();

	this->target_ = this->position_ + this->velocity_;

	// check if target is in bbox
	if ( !this->in_bbox() ) {
		this->target_ = this->position_;
	}

	this->target_.z() = this->drone_op_height_;
}

void DroneController::drone_landing()
{
	auto xy_distance = (this->position_ - this->base_station_pos_)
						   .segment(0, 2)
						   .cwiseAbs()
						   .norm();

	if ( xy_distance < 0.05 ) {
		this->target_ = this->base_station_pos_;
		auto dist =
			(this->position_ - this->base_station_pos_).cwiseAbs().norm();
		if ( dist < 0.05 ) {
			this->flight_mode_ = FlightMode::LANDED;
			this->start_counter_ = 0;
		}
	} else {
		auto over_base_station_pos =
			this->base_station_pos_ +
			Eigen::Vector3d(0, 0, this->drone_op_height_);
		this->target_ = over_base_station_pos;
	}
}

void DroneController::publish_target()
{
	this->publish_target(this->target_);
}

void DroneController::publish_target(const Eigen::Vector3d& pos)
{
	sd_interfaces::msg::FlightTarget target;
	target.pos = eigen_to_sd_pos(pos);
	target.motors_on = true;

	this->pub_target_->publish(target);
}

void DroneController::flight_to_base_station()
{
	this->publish_target(this->base_station_pos_);
}

// subscirber world objects
void DroneController::callback_world_objects(
	const sd_interfaces::msg::WorldObjects& msg)
{
	this->world_objects_ = msg;
}

void DroneController::callback_comm_receive(
	const sd_interfaces::msg::SwarmInfo& msg)
{
	this->swarm_info_ = msg;
	// this->swarm_threats_ = msg.swarm_threats;
}

void DroneController::callback_position(
	const sd_interfaces::msg::PositionStamped::SharedPtr msg)
{
	this->position_ = sd_pos_to_eigen(msg->position);
}

bool DroneController::in_bbox()
{
	return this->in_bbox(this->target_);
}

bool DroneController::in_bbox(const Eigen::Vector3d& pos)
{
	bool between_x =
		(pos.x() <= this->bbox_max_.x()) && (pos.x() >= this->bbox_min_.x());
	bool between_y =
		(pos.y() <= this->bbox_max_.y()) && (pos.y() >= this->bbox_min_.y());
	return (between_x && between_y);
}

} // namespace sd

int main(int argc, char* argv[])
{
	srand(getpid());
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sd::DroneController>());
	rclcpp::shutdown();
	return 0;
}
