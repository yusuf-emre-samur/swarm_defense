#ifndef SWARM_DEFENSE_GAZEBO_ROS_PLUGINS_HELLO_WORLD_HPP_
#define SWARM_DEFENSE_GAZEBO_ROS_PLUGINS_HELLO_WORLD_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rclcpp/rclcpp.hpp>

namespace swarm_defense {
class HelloWorldPlugin : public gazebo::WorldPlugin
{
  public:
	gazebo_ros::Node::SharedPtr ros2node{nullptr};
	/// Constructor
	HelloWorldPlugin();

	~HelloWorldPlugin();

	void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf);
};

} // namespace swarm_defense

#endif
