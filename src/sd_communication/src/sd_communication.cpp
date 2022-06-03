#include "sd_communication/sd_communication.hpp"

namespace sd {
DroneCommunication::DroneCommunication() : rclcpp::Node("DroneCommunication")
{
	// parameters
	// drone id
	this->declare_parameter<uint8_t>("drone_id", 0);
	this->get_parameter("drone_id", this->id_);
	this->name_ = "drone_" + std::to_string(this->id_);

	// old after
	this->declare_parameter<uint8_t>("old_after", 0);
	this->get_parameter("old_after", this->old_after_);

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
	if ( msg->drone_header.drone_id != this->id_ ) {
		auto& drones = this->swarm_positions_.drones;
		auto it = drones.begin();
		bool not_found_flag = true;
		while ( it != drones.end() ) {
			if ( (this->now() - it->stamp) >
				 rclcpp::Duration::from_seconds(this->old_after_) ) {
				it = drones.erase(it);
			}
			if ( it->drone_id == msg->drone_header.drone_id ) {
				not_found_flag = false;
				it->drone_mode = msg->drone_header.drone_mode;
				it->flight_mode = msg->drone_header.flight_mode;
				it->pos = msg->drone_header.pos;
				it->stamp = msg->drone_header.stamp;
			}
			++it;
		}
		if ( not_found_flag ) {
			drones.push_back(msg->drone_header);
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
	this->swarm_info_.swarm_positions = this->swarm_positions_;
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
