#include "sd_communication/sd_communication.hpp"
#include <sd_drone_controller/sd_utils.hpp>

#include <algorithm>
namespace sd {
DroneCommunication::DroneCommunication() : rclcpp::Node("DroneCommunication")
{
	// parameters
	// drone id
	this->declare_parameter<uint8_t>("drone_id", 0);
	this->get_parameter("drone_id", this->id_);
	this->name_ = "drone_" + std::to_string(this->id_);

	// old after
	this->declare_parameter<double>("old_after", 0);
	this->get_parameter("old_after", this->old_after__seconds_);

	this->swarm_info_.pg_score = std::numeric_limits<double>::min();

	// timer
	using namespace std::chrono_literals;
	this->timer_ = rclcpp::create_timer(
		this, this->get_clock(), 100ms,
		std::bind(&DroneCommunication::timer_callback, this));

	// subscriber send
	this->sub_comm_send_ =
		this->create_subscription<sd_interfaces::msg::DroneMsgOut>(
			"communication_send", 1,
			std::bind(&DroneCommunication::callback_comm_send, this,
					  std::placeholders::_1));

	// publisher receive
	this->pub_comm_receive_ =
		this->create_publisher<sd_interfaces::msg::SwarmInfo>(
			"communication_receive", 1);

	// subscriber incoming
	this->sub_comm_incoming_ =
		this->create_subscription<sd_interfaces::msg::DroneMsgOut>(
			"/drones/infos", 1,
			std::bind(&DroneCommunication::callback_comm_incoming, this,
					  std::placeholders::_1));

	// publisher outgoing
	this->pub_comm_outgoing_ =
		this->create_publisher<sd_interfaces::msg::DroneMsgOut>("/drones/infos",
																1);
}

void DroneCommunication::timer_callback()
{
	// this->update_knowledge(time);
	// this->sent_messages_outgoing();

	this->sent_messages_incoming();
	{
		const std::lock_guard<std::mutex> lock(this->swarm_info_mutex_);
		this->swarm_info_.swarm_threats.clear();
	}
	this->last_time_ = this->now();
}

void DroneCommunication::callback_comm_send(
	const sd_interfaces::msg::DroneMsgOut& msg)
{
	this->pub_comm_outgoing_->publish(msg);
}

void DroneCommunication::callback_comm_incoming(
	const sd_interfaces::msg::DroneMsgOut::SharedPtr msg)
{
	const std::lock_guard<std::mutex> lock(this->swarm_info_mutex_);
	//
	// check drone position
	if ( msg->drone_header.drone_id != this->id_ ) {
		auto& drones = this->swarm_info_.swarm_drones;
		auto it = drones.begin();
		bool not_found_flag = true;
		while ( it != drones.end() ) {
			// old information
			if ( (this->now() - it->stamp) >
				 rclcpp::Duration::from_seconds(this->old_after__seconds_) ) {
				it = drones.erase(it);
			}
			// same drone
			if ( it->drone_id == msg->drone_header.drone_id ) {
				not_found_flag = false;
				it->drone_mode = msg->drone_header.drone_mode;
				it->flight_mode = msg->drone_header.flight_mode;
				it->pos = msg->drone_header.pos;
				it->stamp = msg->drone_header.stamp;
			}
			++it;
		}
		// if drone not found
		if ( not_found_flag ) {
			drones.push_back(msg->drone_header);
		}

		//
		// threats
		// iterate over threats and copy
		if ( this->swarm_threats_.size() > 0 ) {
			bool not_found;
			// threats not empty
			// iterate over other detected threats
			for ( const auto& threat : msg->detected_threats ) {
				not_found = true;
				for ( auto& e_threat : this->swarm_threats_ ) {
					auto dist = (sd_pos_to_eigen(e_threat.pos) -
								 sd_pos_to_eigen(threat.pos))
									.cwiseAbs()
									.norm();
					if ( dist < 2.0 ) {
						// received threat is same like existing
						// add following ids to existing threat, if not already
						// in list
						if ( std::find_if(e_threat.following_drones_id.begin(),
										  e_threat.following_drones_id.end(),
										  [&](const uint8_t& drone_id) {
											  return drone_id ==
													 threat.detected_drone_id;
										  }) ==
							 e_threat.following_drones_id.end() ) {
							// id not found in list, append to list
							e_threat.following_drones_id.push_back(
								threat.detected_drone_id);
						}

						// update position and time information
						e_threat.pos = threat.pos;
						e_threat.time_detected = threat.time_detected;

						not_found = false;
					}
				}
				if ( not_found ) {
					this->swarm_threats_.push_back(threat);
				}
			}
		} else if ( msg->detected_threats.size() > 0 ) {
			// both lists not empty
			this->swarm_threats_ = msg->detected_threats;
		}

		// remove old threats
		auto& s_threats = this->swarm_threats_;
		s_threats.erase(
			std::remove_if(s_threats.begin(), s_threats.end(),
						   [&](const sd_interfaces::msg::Threat& threat) {
							   return ((this->now() -
											rclcpp::Time(threat.time_detected) >
										rclcpp::Duration::from_seconds(5)));
						   }),
			s_threats.end());

		// PSO pg check, if drones pb > pg, update pg score and position
		if ( msg->pb_score < this->swarm_info_.pg_score ) {
			this->swarm_info_.pg_score = msg->pb_score;
			this->swarm_info_.pg = msg->pb;
		}
	}
}

void DroneCommunication::sent_messages_outgoing()
{
	if ( this->msg_send_ ) {
		this->pub_comm_outgoing_->publish(*this->msg_send_.get());
	}
}

void DroneCommunication::update_knowledge()
{
}

void DroneCommunication::sent_messages_incoming()
{
	this->swarm_info_.swarm_threats = this->swarm_threats_;
	this->pub_comm_receive_->publish(this->swarm_info_);
}

} // namespace sd

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sd::DroneCommunication>());
	rclcpp::shutdown();
	return 0;
}
