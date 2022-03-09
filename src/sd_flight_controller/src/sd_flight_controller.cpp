#include "sd_flight_controller/sd_flight_controller.hpp"

using namespace std::chrono_literals;
namespace sd {
namespace ros {
FlightController::FlightController() : Node("default_node_name")
{
	// node params
	// topic name of pose subscriper
	this->declare_parameter<std::string>("pose_topic_name", "");
	this->get_parameter("pose_topic_name", this->pose_sub_topic_name_);
	// topic name of rpm publisher
	this->declare_parameter<std::string>("rpm_topic_name", "");
	this->get_parameter("rpm_topic_name", this->rpm_pub_topic_name_);
	// pid
	double kp, ki, kd;
	this->declare_parameter<double>("kp", 0.0);
	this->declare_parameter<double>("ki", 0.0);
	this->declare_parameter<double>("kd", 0.0);
	this->get_parameter("kp", kp);
	this->get_parameter("ki", ki);
	this->get_parameter("kd", kd);

	// pose subscriber
	this->pose_sub_ =
		this->create_subscription<geometry_msgs::msg::PoseStamped>(
			this->pose_sub_topic_name_, 1,
			std::bind(&FlightController::on_pose_msg_callback, this,
					  std::placeholders::_1));

	// pid timer
	this->pid_timer_ = this->create_wall_timer(
		10ms, std::bind(&FlightController::pid_timer_callback, this));

	// rpm publisher
	this->rmp_pub_ = this->create_publisher<sd_interfaces::msg::QuadcopterRPM>(
		this->rpm_pub_topic_name_, 10);

	// z pid
	this->pid_z = std::make_unique<PID>(10, -100, 100, kp, ki, kd);
	this->rpm_thrust = 0;

	this->last_time_ = this->now();
	// INFO
	RCLCPP_INFO(this->get_logger(), "SD Flight Controller Node started!");
}

// callback for each received pose msg
void FlightController::on_pose_msg_callback(
	geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	// simply update last received msg
	this->last_pose_ = msg;
}

// timer callback for pid controller
void FlightController::pid_timer_callback()
{
	auto time = this->now();
	// do pid
	if ( this->last_pose_ != nullptr && (time != this->last_time_) ) {
		this->rpm_thrust =
			pid_z->calculate(1, this->last_pose_->pose.position.z);
		RCLCPP_INFO(this->get_logger(),
					std::to_string(this->rpm_thrust).c_str());
	}
	// calc motor rpms

	sd_interfaces::msg::QuadcopterRPM msg;
	msg.header.frame_id = this->get_name();
	msg.header.stamp = this->now();
	msg.rotor0.rpm = (2099 + this->rpm_thrust);
	msg.rotor1.rpm = -(2099 + this->rpm_thrust);
	msg.rotor2.rpm = (2099 + this->rpm_thrust);
	msg.rotor3.rpm = -(2099 + this->rpm_thrust);
	// publish rpm message
	this->rmp_pub_->publish(msg);
	this->last_time_ = time;
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