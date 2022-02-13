#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sd_gazebo_ros_plugins/hello_world_plugin.hpp>

namespace sd {
namespace gazebo_ros_plugins {

HelloWorldPlugin::HelloWorldPlugin() : gazebo::WorldPlugin()
{
}

HelloWorldPlugin::~HelloWorldPlugin()
{
	RCLCPP_INFO(ros2node->get_logger(), "desctructor of Hello World plugin");
	ros2node.reset();
}

void HelloWorldPlugin::Load(gazebo::physics::WorldPtr _world,
							sdf::ElementPtr _sdf)
{
	ros2node = gazebo_ros::Node::Get(_sdf);
	RCLCPP_INFO(ros2node->get_logger(), "Hello World plugin have been loaded!");
}

GZ_REGISTER_WORLD_PLUGIN(HelloWorldPlugin)

} // namespace gazebo_ros_plugins
} // namespace sd
