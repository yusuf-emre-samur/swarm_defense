#ifndef SD_GAZEBO_ROS_PLUGINS_DRONE_PLUGIN_HPP_
#define SD_GAZEBO_ROS_PLUGINS_DRONE_PLUGIN_HPP_
// cpp
#include <array>

// gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Pose3.hh>
// ros
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
// msgs
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sd_interfaces/msg/rotor_rpm.hpp>
#include <sd_interfaces/msg/rpm.hpp>
#include <std_msgs/msg/string.hpp>

namespace sd {
namespace gazebo_ros_plugins {

class DronePlugin : public gazebo::ModelPlugin
{
  public:
	// Constructor
	DronePlugin();

	~DronePlugin();

	void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	void OnUpdate(const gazebo::common::UpdateInfo& _info);
	void topic_callback(sd_interfaces::msg::RotorRPM::SharedPtr msg);

	//
	void updateThrust();
	double calculateThrust(const double& w);
	double calculateTorque(const double& w);
	double rpm_to_rad_p_sec(const int& rpm);

	// imu + gps = pose
	void publish_pose();

  private:
	// gazebo
	gazebo::physics::ModelPtr model_;
	gazebo::event::ConnectionPtr updateConnection_;
	ignition::math::Pose3d pose_;
	// imu sensor
	gazebo::sensors::SensorPtr imu_sensor_;
	gazebo::sensors::ImuSensorPtr imu_;
	// pose pub
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

	// ros
	gazebo_ros::Node::SharedPtr ros2node_;
	rclcpp::Subscription<sd_interfaces::msg::RotorRPM>::SharedPtr subscription_;

	// params
	const uint num_rotors_ = 4;
	std::array<std::string, 4> rotor_link_names_;
	std::array<int16_t, 4> rotor_rpms_;
	std::string imu_link_name_;
	std::string imu_sensor_name_;

	double rate_;
	double rotor_thrust_coeff_;
	double rotor_torque_coeff_;
	std::mutex lock_;
};
} // namespace gazebo_ros_plugins
} // namespace sd

#endif
