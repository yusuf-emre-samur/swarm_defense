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
	this->declare_parameter<double>("z_kp", 0.0);
	this->declare_parameter<double>("z_ki", 0.0);
	this->declare_parameter<double>("z_kd", 0.0);
	this->get_parameter("z_kp", kp);
	this->get_parameter("z_ki", ki);
	this->get_parameter("z_kd", kd);

	// pose subscriber
	this->pose_sub_ =
		this->create_subscription<geometry_msgs::msg::PoseStamped>(
			this->pose_sub_topic_name_, 1,
			std::bind(&FlightController::on_pose_msg_callback, this,
					  std::placeholders::_1));

	// pid timer
	this->pid_timer_ = this->create_wall_timer(
		1ms, std::bind(&FlightController::pid_timer_callback, this));

	// rpm publisher
	this->rmp_pub_ = this->create_publisher<sd_interfaces::msg::QuadcopterRPM>(
		this->rpm_pub_topic_name_, 10);

	// z pid
	this->pid_z_ = std::make_unique<PID>(1, -100, 100, kp, ki, kd);
	this->pid_roll_ = std::make_unique<PID>(1, -10, 10, 0.1, 0.0, 0.00001);
	this->pid_pitch_ = std::make_unique<PID>(1, -10, 10, 0.1, 0.0, 0.00001);
	this->pid_yaw_ = std::make_unique<PID>(1, -10, 10, 0.1, 0.0, 0.00001);
	this->rpm_thrust_ = 0;
	this->rpm_roll_ = 0;
	this->rpm_pitch_ = 0;
	this->rpm_yaw_ = 0;

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
		// thrust
		this->rpm_thrust_ =
			this->pid_z_->calculate(1, this->last_pose_->pose.position.z);
		// roll
		this->rpm_roll_ =
			this->pid_roll_->calculate(0, this->last_pose_->pose.orientation.x);
		// pitch
		this->rpm_pitch_ = this->pid_pitch_->calculate(
			0, this->last_pose_->pose.orientation.y);
		// yaw
		this->rpm_yaw_ =
			this->pid_yaw_->calculate(0, this->last_pose_->pose.orientation.z);
	}
	this->motor_mixing();
	this->last_time_ = time;
}

void FlightController::motor_mixing() const
{
	sd_interfaces::msg::QuadcopterRPM msg;
	msg.header.frame_id = this->get_name();
	msg.header.stamp = this->now();
	msg.rotor0.rpm = -(2099 + static_cast<int>(rpm_thrust_ + rpm_yaw_ -
											   rpm_pitch_ - rpm_roll_));
	msg.rotor1.rpm = -(2099 + static_cast<int>(rpm_thrust_ + rpm_yaw_ +
											   rpm_pitch_ + rpm_roll_));
	msg.rotor2.rpm = (2099 + static_cast<int>(rpm_thrust_ - rpm_yaw_ -
											  rpm_pitch_ + rpm_roll_));
	msg.rotor3.rpm = (2099 + static_cast<int>(rpm_thrust_ - rpm_yaw_ +
											  rpm_pitch_ - rpm_roll_));
	// publish rpm message
	this->rmp_pub_->publish(msg);
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