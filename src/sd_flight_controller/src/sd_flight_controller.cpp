#include "sd_flight_controller/sd_flight_controller.hpp"

namespace sd {
FlightController::FlightController() : rclcpp::Node("FlightController")
{
	// parameters
	// drone id
	this->declare_parameter<int>("drone_id", 0);
	this->get_parameter("drone_id", this->id_);
	this->name_ = "drone_" + std::to_string(this->id_);

	// drone base station
	this->declare_parameter("base_station_pos");
	rclcpp::Parameter base_station_pos("base_station_pos",
									   std::vector<double>({}));
	this->get_parameter("base_station_pos", base_station_pos);
	auto tmp = base_station_pos.as_double_array();

	using namespace std::chrono_literals;
	this->timer_ = rclcpp::create_timer(
		this, this->get_clock(), 250ms,
		std::bind(&FlightController::timer_callback, this));

	// subscription target
	this->sub_target_ = this->create_subscription<sd_interfaces::msg::Position>(
		"target", 10,
		std::bind(&FlightController::callback_target, this,
				  std::placeholders::_1));

	// client set target
	this->client_set_target_ =
		this->create_client<sd_interfaces::srv::SetDroneTarget>("set_target");
}

void FlightController::timer_callback()
{
	if ( this->target_set_ ) {
		this->set_target();
	}
}

void FlightController::set_target()
{
	auto request =
		std::make_shared<sd_interfaces::srv::SetDroneTarget::Request>();
	request->x = this->target_.x();
	request->y = this->target_.y();
	request->z = this->target_.z();

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

void FlightController::callback_target(const sd_interfaces::msg::Position& msg)
{
	// set target
	this->target_.x() = msg.x;
	this->target_.y() = msg.y;
	this->target_.z() = msg.z;

	{
		std::lock_guard<std::mutex> lock(this->target_set_m);
		this->target_set_ = true;
	}
}

} // namespace sd

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<sd::FlightController>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
