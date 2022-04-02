#include "sd_actor_controller/sd_actor_controller.hpp"

using namespace std::chrono_literals;
namespace sd {
namespace ros {
ActorController::ActorController() : Node("default_node_name")
{
	// node params
	// topic name of pose subscriper
	this->declare_parameter<std::string>("pos_sub_topic_name", "");
	this->get_parameter("pos_sub_topic_name", this->pos_sub_topic_name_);
	// topic name of rpm publisher
	this->declare_parameter<std::string>("pos_pub_topic_name", "");
	this->get_parameter("pos_pub_topic_name", this->pos_pub_topic_name_);
	// pid

	// pose subscriber
	this->pos_sub_ =
		this->create_subscription<sd_interfaces::msg::Position3Stamped>(
			this->pos_sub_topic_name_, 1,
			std::bind(&ActorController::on_position_msg_callback, this,
					  std::placeholders::_1));

	// pid timer
	this->flight_controller_timer_ = this->create_wall_timer(
		100ms,
		std::bind(&ActorController::flight_controller_timer_callback, this));

	// rpm publisher
	this->pos_pub_ =
		this->create_publisher<sd_interfaces::msg::Position3Stamped>(
			this->pos_pub_topic_name_, 10);

	this->last_time_ = this->now();

	// INFO
	RCLCPP_INFO(this->get_logger(), "SD Flight Controller Node started!");
}

// timer callback for flight controller loop
void ActorController::flight_controller_timer_callback()
{
	auto time = this->now();
	// this->publish_goal_pos();
	this->last_time_ = time;
}

// callback for each received pose msg
void ActorController::on_position_msg_callback(
	const sd_interfaces::msg::Position3Stamped::SharedPtr msg)
{
	// simply update last received msg
	this->last_pos_ = msg->pos3;
}

void ActorController::publish_goal_position() const
{
	sd_interfaces::msg::Position3Stamped msg;
	// header
	msg.header.frame_id = this->get_name();
	msg.header.stamp = this->now();
	// pos3
	msg.pos3.set__x(this->target_pos_.x);
	msg.pos3.set__y(this->target_pos_.y);
	msg.pos3.set__z(this->target_pos_.z);

	this->pos_pub_->publish(msg);
}

void ActorController::set_goal_position(
	const sd_interfaces::msg::Position3Stamped::SharedPtr pos)
{
	this->target_pos_ = pos->pos3;
}

} // namespace ros
} // namespace sd

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sd::ros::ActorController>());
	rclcpp::shutdown();
	return 0;
}