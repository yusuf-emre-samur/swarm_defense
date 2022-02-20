#ifndef SD_GAZEBO_ROS_PLUGINS_DRONE_PLUGIN_HPP_
#define SD_GAZEBO_ROS_PLUGINS_DRONE_PLUGIN_HPP_
// cpp
#include <vector>

// gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <ignition/math/Pose3.hh>
// ros
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
// msgs
#include <geometry_msgs/msg/vector3.hpp>
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
	void OnUpdate();
	void topic_callback(geometry_msgs::msg::Vector3::SharedPtr msg);

	//
	void updateThrust();
	double calculateThrust(const double& w);
	double calculateTorque(const double& w);

  private:
	// gazebo
	gazebo::physics::ModelPtr model_;
	gazebo::event::ConnectionPtr updateConnection_;
	std::mutex pose_mtx_;
	ignition::math::Pose3d pose_;

	// ros
	gazebo_ros::Node::SharedPtr ros2node_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;

	// params
	std::vector<std::string> rotor_link_names_;
	double rate_;
	double rotor_thrust_coeff_;
	double rotor_torque_coeff_;
	std::mutex lock_;
};
} // namespace gazebo_ros_plugins
} // namespace sd

#endif
