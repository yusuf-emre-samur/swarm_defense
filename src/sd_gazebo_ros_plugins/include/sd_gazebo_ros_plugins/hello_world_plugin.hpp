#ifndef SD_GAZEBO_ROS_PLUGINS_HELLO_WORLD_HPP_
#define SD_GAZEBO_ROS_PLUGINS_HELLO_WORLD_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sd {
namespace gazebo_ros_plugins {

class HelloWorldPlugin : public gazebo::WorldPlugin
{
  public:
	gazebo_ros::Node::SharedPtr ros2node{nullptr};
	/// Constructor
	HelloWorldPlugin();

	~HelloWorldPlugin();

	void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf);
};
} // namespace gazebo_ros_plugins
} // namespace sd

#endif
