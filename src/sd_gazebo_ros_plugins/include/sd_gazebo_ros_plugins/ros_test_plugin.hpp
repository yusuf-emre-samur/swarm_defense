#ifndef SD_GAZEBO_ROS_PLUGINS_ROS_TEST_HPP_
#define SD_GAZEBO_ROS_PLUGINS_ROS_TEST_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>

// ros
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
// msgs
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/string.hpp>

namespace sd {
namespace gazebo_ros_plugins {

class RosTestPlugin : public gazebo::ModelPlugin
{
  public:
	// Constructor
	RosTestPlugin();

	~RosTestPlugin();

	void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	void OnUpdate();
	void topic_callback(geometry_msgs::msg::Vector3::SharedPtr msg);

  private:
	// gazebo
	gazebo::physics::ModelPtr model_;
	gazebo::event::ConnectionPtr updateConnection_;

	// ros
	gazebo_ros::Node::SharedPtr ros2node_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
};
} // namespace gazebo_ros_plugins
} // namespace sd

#endif
