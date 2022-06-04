#include "sd_drone_controller/sd_drone_controller.hpp"
#include <random>
std::default_random_engine generator;
std::uniform_real_distribution<double> distribution(0.0, 1.0);

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
	auto tmp = base_station_pos.as_double_array();
	this->base_station_pos_.x() = tmp[0];
	this->base_station_pos_.y() = tmp[1];
	this->base_station_pos_.z() = tmp[2];

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
	auto tmp2 = pso_max_vel.as_double_array();
	this->max_velocity_.x() = tmp2[0];
	this->max_velocity_.y() = tmp2[1];
	this->max_velocity_.z() = tmp2[2];

	// set pb and pg to max
	this->pb_score_ = std::numeric_limits<double>::max();
	this->pg_score_ = std::numeric_limits<double>::max();

	if ( this->battery_ < 80.0 ) {
		// start at e
		this->drone_mode_ = DroneMode::NOTREADY;
	} else {
		// start at a
		this->drone_mode_ = DroneMode::READY;
	}
	this->flight_mode_ = FlightMode::LANDED;

	this->drone_op_height_ = static_cast<double>(this->id_) * 1.5 + 15;

	using namespace std::chrono_literals;
	this->timer_ =
		rclcpp::create_timer(this, this->get_clock(), 100ms,
							 std::bind(&DroneController::timer_callback, this));

	// ros subscriber
	this->sub_world_objects_ =
		this->create_subscription<sd_interfaces::msg::WorldObjects>(
			"/world/objects", 1,
			std::bind(&DroneController::callback_world_objects, this,
					  std::placeholders::_1));

	this->sub_comm_receive_ =
		this->create_subscription<sd_interfaces::msg::SwarmInfo>(
			"communication_receive", 1,
			std::bind(&DroneController::callback_comm_receive, this,
					  std::placeholders::_1));

	this->sub_position_ =
		this->create_subscription<sd_interfaces::msg::PositionStamped>(
			"position", 1,
			std::bind(&DroneController::callback_position, this,
					  std::placeholders::_1));

	this->pub_comm_send_ = create_publisher<sd_interfaces::msg::DroneMsgOut>(
		"communication_send", 1);

	// ros publisher
	this->pub_target_ =
		this->create_publisher<sd_interfaces::msg::FlightTarget>("target", 1);
}

void DroneController::timer_callback()
{
	this->share_knowledge_to_swarm();
	this->process_swarm_information();
	this->simulate_battery();
	this->detect_threats();
	this->flight();
	// this->filter_detected_threats();
}

void DroneController::si_algorithms()
{
	this->calculate_pso_velocity();
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

	this->pub_comm_send_->publish(msg);
}

void DroneController::process_swarm_information()
{
	if ( this->drone_mode_ != DroneMode::NOTREADY ) {
		this->check_start();
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
		this->drone_start();
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

void DroneController::check_start()
{
	const int8_t num_drones_needed = this->min_flying_drones_;
	// check how many drones are in flying mode
	int8_t num_drones_landed_ready = 0;
	int8_t num_drones_flying_or_starting = 0;
	int8_t num_drones_lower_id = 0;

	DroneMode drone_mode;
	FlightMode flight_mode;
	for ( const auto& drone : this->swarm_info_.swarm_positions.drones ) {

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

void DroneController::drone_start()
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
	this->target_.z() = this->drone_op_height_;
}

void DroneController::drone_landed()
{
	// turn motors off
	sd_interfaces::msg::FlightTarget target;
	target.motors_on = false;
	this->pub_target_->publish(target);
	this->start_counter_ = 0;
}

void DroneController::drone_landing()
{

	auto pos_wo_z = this->position_;
	pos_wo_z.z() = 0;

	auto pos_bs_wo_z = this->base_station_pos_;
	pos_bs_wo_z.z() = 0;

	auto xy_distance = (pos_wo_z - pos_bs_wo_z).cwiseAbs().norm();

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

void DroneController::turn_motors_off()
{
	sd_interfaces::msg::FlightTarget target;
	target.motors_on = false;

	this->pub_target_->publish(target);
}

void DroneController::detect_threats()
{
	for ( const auto& object : this->world_objects_.objects ) {
		auto obj_pos = Eigen::Vector3d(
			object.bbox.center.x, object.bbox.center.y, object.bbox.center.z);

		auto dist = (this->position_ - obj_pos).cwiseAbs().norm();

		if ( dist < this->perception_radius_ ) {
		}
	}
}

void DroneController::filter_detected_threats()
{
}

void DroneController::calculate_pso_velocity()
{
	auto r1 = distribution(generator);
	auto r2 = distribution(generator);

	this->velocity_ = (this->w_ * this->velocity_ +
					   r1 * this->c1_ * (this->pb_ - this->position_) +
					   r2 * this->c2_ * (this->pg_ - this->position_))
						  .normalized();

	this->velocity_ = this->velocity_.array() * this->max_velocity_;
}

void DroneController::publish_target()
{
	this->publish_target(this->target_);
}

void DroneController::publish_target(const Eigen::Vector3d& pos)
{
	sd_interfaces::msg::FlightTarget target;
	target.pos.x = pos.x();
	target.pos.y = pos.y();
	target.pos.z = pos.z();
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

// subscriber communincation receive
void DroneController::callback_comm_receive(
	const sd_interfaces::msg::SwarmInfo& msg)
{
	this->swarm_info_ = msg;
}

// subscriber position
void DroneController::callback_position(
	const sd_interfaces::msg::PositionStamped::SharedPtr msg)
{
	this->position_.x() = msg->position.x;
	this->position_.y() = msg->position.y;
	this->position_.z() = msg->position.z;
}

} // namespace sd

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sd::DroneController>());
	rclcpp::shutdown();
	return 0;
}
