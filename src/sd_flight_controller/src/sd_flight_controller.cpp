#include "sd_flight_controller/sd_flight_controller.hpp"

using namespace std::chrono_literals;
namespace sd {
namespace ros {
FlightController::FlightController() : Node("default_node_name")
{
	// node params
	// topic name of pose subscriper
	this->declare_parameter<std::string>("pose_sub_topic_name", "");
	this->get_parameter("pose_sub_topic_name", this->pose_sub_topic_name_);
	// topic name of rpm publisher
	this->declare_parameter<std::string>("pose_pub_topic_name", "");
	this->get_parameter("pose_pub_topic_name", this->pose_pub_topic_name_);
	// pid

	// pose subscriber
	this->pose_sub_ =
		this->create_subscription<geometry_msgs::msg::PoseStamped>(
			this->pose_sub_topic_name_, 1,
			std::bind(&FlightController::on_pose_msg_callback, this,
					  std::placeholders::_1));

	// pid timer
	this->flight_controller_timer_ = this->create_wall_timer(
		100ms,
		std::bind(&FlightController::flight_controller_timer_callback, this));

	// rpm publisher
	this->pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
		this->pose_pub_topic_name_, 10);

	this->last_time_ = this->now();

	// INFO
	RCLCPP_INFO(this->get_logger(), "SD Flight Controller Node started!");
}

// timer callback for flight controller loop
void FlightController::flight_controller_timer_callback()
{
	auto time = this->now();
	// this->publish_goal_pos();
	this->last_time_ = time;
}

// callback for each received pose msg
void FlightController::on_pose_msg_callback(
	geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	// simply update last received msg
	this->last_pose_ = msg;
}

void FlightController::publish_goal_pos() const
{
	geometry_msgs::msg::PoseStamped msg;
	msg.pose.position.set__x(this->goal_pose_.position.x);
	msg.pose.position.set__y(this->goal_pose_.position.y);
	msg.pose.position.set__z(this->goal_pose_.position.z);
	msg.pose.orientation.set__w(this->goal_pose_.orientation.w);
	msg.pose.orientation.set__x(this->goal_pose_.orientation.x);
	msg.pose.orientation.set__y(this->goal_pose_.orientation.y);
	msg.pose.orientation.set__z(this->goal_pose_.orientation.z);

	msg.header.frame_id = this->get_name();
	msg.header.stamp = this->now();
	this->pose_pub_->publish(msg);
}

void FlightController::set_goal_pos(geometry_msgs::msg::Pose::SharedPtr pos)
{
	this->goal_pose_.position = pos->position;
	this->goal_pose_.orientation = pos->orientation;
}

} // namespace ros
} // namespace sd

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<sd::ros::FlightController>());
	rclcpp::shutdown();
	return 0;
}