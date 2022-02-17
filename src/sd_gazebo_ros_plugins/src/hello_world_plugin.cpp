#include <sd_gazebo_ros_plugins/hello_world_plugin.hpp>

namespace sd {
namespace gazebo_ros_plugins {

HelloWorldPlugin::HelloWorldPlugin() : gazebo::WorldPlugin()
{
}

HelloWorldPlugin::~HelloWorldPlugin()
{
	ros2node_.reset();
}

void HelloWorldPlugin::Load(gazebo::physics::WorldPtr _world,
							sdf::ElementPtr _sdf)
{
	ros2node_ = gazebo_ros::Node::Get(_sdf);
	RCLCPP_INFO(ros2node_->get_logger(), "Loaded HelloWorldPlugin!");
}

GZ_REGISTER_WORLD_PLUGIN(HelloWorldPlugin)
} // namespace gazebo_ros_plugins
} // namespace sd