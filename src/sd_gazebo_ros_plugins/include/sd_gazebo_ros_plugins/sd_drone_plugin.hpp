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
#include <std_msgs/msg/string.hpp>
// other

namespace sd {
namespace gazebo_ros_plugins {

class DronePlugin : public gazebo::ModelPlugin
{
  public:
	// con. / decon.
	DronePlugin();
	~DronePlugin();

	// gazebo plugin functions
	void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	void OnUpdate(const gazebo::common::UpdateInfo& _info);

  private: // functions
		   // called when on new pose on topic
	void on_pose_msg_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
	// quadcopter flight functions
	void fakeRotation();

	// simulate fake fly
	void fakeFly();
	void cropVelocity();

	// ros functions
	// imu + gps = pose
	void publish_pose() const;

  private: // vars
	// gazebo
	gazebo::physics::ModelPtr model_;
	gazebo::event::ConnectionPtr update_callback_;
	gazebo::common::Time last_time_;
	// current pose and goal pose
	ignition::math::Pose3d pose_;
	ignition::math::Pose3d goal_pose_;

	// imu sensor
	gazebo::sensors::ImuSensorPtr imu_;

	// curr pose pub
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

	// ros node
	gazebo_ros::Node::SharedPtr ros2node_;

	// goal pose subscriber
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

	// vel
	ignition::math::Vector3d vel_;
	ignition::math::Vector3d ang_vel_;
	ignition::math::Vector3d max_vel_;
	ignition::math::Vector3d max_ang_vel_;

	// for fake rotor rotation
	static constexpr uint num_rotors_ = 4;
	std::array<std::string, num_rotors_> rotor_link_names_;
};
} // namespace gazebo_ros_plugins
} // namespace sd

#endif
