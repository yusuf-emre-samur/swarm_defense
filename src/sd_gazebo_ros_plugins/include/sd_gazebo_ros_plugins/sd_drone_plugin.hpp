#ifndef SD_GAZEBO_ROS_PLUGINS_DRONE_PLUGIN_HPP_
#define SD_GAZEBO_ROS_PLUGINS_DRONE_PLUGIN_HPP_
// cpp
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <random>
#include <string>

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
	void OnUpdate();

  private: // functions
		   // called when on new pose on topic
	void on_pose_msg_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

	// ros functions
	// imu + gps = pose
	void publish_pose() const;

	// simulate fake fly
	void fakeFly();
	// crop velocity to [-max,max]
	void cropVelocity();
	// create pitch and roll for motion
	ignition::math::Vector3d fakeAngularMotion() const;
	// add gaussian noise to velocity
	ignition::math::Pose3d getGaussianNoise() const;
	// quadcopter flight functions
	void fakeRotation();

	// gazebo
	gazebo::physics::ModelPtr model_;
	gazebo::event::ConnectionPtr update_callback_;
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
	ignition::math::Vector3d last_vel_;
	ignition::math::Vector3d ang_vel_;
	ignition::math::Vector3d max_vel_;
	ignition::math::Vector3d max_ang_vel_;

	// for fake rotor rotation
	static constexpr uint num_rotors_ = 4;
	std::array<std::string, num_rotors_> rotor_link_names_;

	// gilt
	std::string gimbal_tilt_link_;
};
} // namespace gazebo_ros_plugins
} // namespace sd

#endif
