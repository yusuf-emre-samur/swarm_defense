#ifndef SWARM_DEFENSE_GAZEBO_ROS_PLUGINS_HELLO_WORLD_HPP_
#define SWARM_DEFENSE_GAZEBO_ROS_PLUGINS_HELLO_WORLD_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <rclcpp/rclcpp.hpp>

namespace gazebo {
class HelloWorldPlugin : public WorldPlugin
{
  public:
	rclcpp::Node ::SharedPtr ros2node;
	/// Constructor
	HelloWorldPlugin() : WorldPlugin()
	{
		if ( !rclcpp::ok() ) {
			int argc = 0;
			char** argv = NULL;
			rclcpp::init(argc, argv);
		}
		ros2node = rclcpp::Node::make_shared("test_node");

		RCLCPP_INFO(ros2node->get_logger(), "Hello World!");
	}

	void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
	{
	}
};

GZ_REGISTER_WORLD_PLUGIN(HelloWorldPlugin)

} // namespace gazebo

#endif
