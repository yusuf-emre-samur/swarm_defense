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
// interfaces
#include <sd_interfaces/msg/position_stamped.hpp>
#include <std_msgs/msg/header.hpp>
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
	void on_position_msg_callback(
		const sd_interfaces::msg::PositionStamped::SharedPtr msg);

	// ros functions
	// imu + gps = pose
	void publish_position() const;

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

	std::string id_;

	// gazebo
	gazebo::physics::ModelPtr model_;
	gazebo::event::ConnectionPtr update_callback_;

	// current and target position
	ignition::math::Vector3d pos_;
	ignition::math::Vector3d target_pos_;

	// ros
	gazebo_ros::Node::SharedPtr ros2node_;

	// curr pose pub
	rclcpp::Publisher<sd_interfaces::msg::PositionStamped>::SharedPtr pos_pub_;

	// target pose subscriber
	rclcpp::Subscription<sd_interfaces::msg::PositionStamped>::SharedPtr
		target_pos_sub_;

	// vel
	ignition::math::Vector3d vel_;
	ignition::math::Vector3d last_vel_;
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
