#include "sd_flight_controller/sd_flight_controller.hpp"

namespace sd {
FlightController::FlightController() : rclcpp::Node("FlightController")
{
	// parameters
	// drone id
	this->declare_parameter<uint8_t>("drone_id", 0);
	this->get_parameter("drone_id", this->drone_id_);
	this->name_ = "drone_" + std::to_string(this->drone_id_);

	using namespace std::chrono_literals;
	this->timer_ = rclcpp::create_timer(
		this, this->get_clock(), 250ms,
		std::bind(&FlightController::timer_callback, this));

	// subscription target
	this->sub_target_ =
		this->create_subscription<sd_interfaces::msg::FlightTarget>(
			"target", 10,
			std::bind(&FlightController::callback_target, this,
					  std::placeholders::_1));

	// subscribe communication receive
	this->sub_comm_receive_ =
		this->create_subscription<sd_interfaces::msg::SwarmInfo>(
			"communication_receive", 1,
			std::bind(&FlightController::callback_comm_receive, this,
					  std::placeholders::_1));

	// subscribe positiom
	this->sub_pos_ =
		this->create_subscription<sd_interfaces::msg::PositionStamped>(
			"position", 1,
			std::bind(&FlightController::callback_position, this,
					  std::placeholders::_1));

	// client set target
	this->client_set_target_ =
		this->create_client<sd_interfaces::srv::SetDroneTarget>("set_target");
}

void FlightController::timer_callback()
{
	this->check_collision();
	if ( this->target_set_ ) {
		this->set_target();
	}
}

void FlightController::set_target()
{
	auto request =
		std::make_shared<sd_interfaces::srv::SetDroneTarget::Request>();

	request->target.pos.x = this->target_.x();
	request->target.pos.y = this->target_.y();
	request->target.pos.z = this->target_.z();
	request->target.motors_on = this->motors_on_;

	using ServiceResponseFuture =
		rclcpp::Client<sd_interfaces::srv::SetDroneTarget>::SharedFuture;

	auto response_received_callback = [this](ServiceResponseFuture future) {
		auto result = future.get();
		if ( result->success ) {
			{
				std::lock_guard<std::mutex> lock(this->target_set_m);
				this->target_set_ = false;
			}
		} else {
			RCLCPP_ERROR(this->get_logger(),
						 "set_target request not successfully");
		}
	};
	auto future_result = this->client_set_target_->async_send_request(
		request, response_received_callback);
}

void FlightController::callback_target(
	const sd_interfaces::msg::FlightTarget& msg)
{
	// set target
	this->target_.x() = msg.pos.x;
	this->target_.y() = msg.pos.y;
	this->target_.z() = msg.pos.z;
	this->motors_on_ = msg.motors_on;
	{
		std::lock_guard<std::mutex> lock(this->target_set_m);
		this->target_set_ = true;
	}
}

void FlightController::callback_comm_receive(
	const sd_interfaces::msg::SwarmInfo& msg)
{
	this->swarm_positions_ = msg.swarm_positions;
}

void FlightController::check_collision()
{
	for ( const auto& drone : this->swarm_positions_.drones ) {
		if ( static_cast<FlightMode>(drone.flight_mode) !=
			 FlightMode::LANDED ) {
			auto pos_other_drone =
				Eigen::Vector3d(drone.pos.x, drone.pos.y, drone.pos.z);
			auto distance =
				(this->position_ - pos_other_drone).cwiseAbs().norm();
			// if distance is smaller than 3m and drone has lower id
			if ( distance < 1.5 ) {
				this->target_ = this->position_;
			}
		}
	}
}

void FlightController::callback_position(
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
	rclcpp::spin(std::make_shared<sd::FlightController>());
	rclcpp::shutdown();
	return 0;
}
