#ifndef SWARM_DEFENSE_GAZEBO_ROS_PLUGINS_HELLO_WORLD_HPP_
#define SWARM_DEFENSE_GAZEBO_ROS_PLUGINS_HELLO_WORLD_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo {
class HelloWorldPlugin : public WorldPlugin
{
  public:
	gazebo_ros::Node::SharedPtr ros2node{nullptr};
	/// Constructor
	HelloWorldPlugin() : WorldPlugin()
	{
	}

	~HelloWorldPlugin()
	{
		RCLCPP_INFO(ros2node->get_logger(), "desctructor");
		ros2node.reset();
	}

	void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
	{
		ros2node = gazebo_ros::Node::Get(_sdf);
		RCLCPP_INFO(ros2node->get_logger(), "Hello World! from load");
	}
};

GZ_REGISTER_WORLD_PLUGIN(HelloWorldPlugin)

} // namespace gazebo

#endif
