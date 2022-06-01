#ifndef SD_WORLD_PLUGIN_HPP_
#define SD_WORLD_PLUGIN_HPP_

// std
#include <algorithm>

// gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
// ros
#include <rclcpp/rclcpp.hpp>
// interfaces
#include <sd_interfaces/msg/world_objects.hpp>

namespace sd {

class WorldPlugin : public gazebo::WorldPlugin
{
  public:
	WorldPlugin();
	~WorldPlugin();
	void OnUpdate();

	void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf);

  private:
	// world ptr
	gazebo::physics::WorldPtr world_;
	// callback
	gazebo::event::ConnectionPtr updateConnection_;
	// ros node
	gazebo_ros::Node::SharedPtr ros2node_;
	// publisher
	rclcpp::Publisher<sd_interfaces::msg::WorldObjects>::SharedPtr publisher_;

	// actor and drone names
	std::vector<std::string> track_objects_;
};

} // namespace sd

#endif
