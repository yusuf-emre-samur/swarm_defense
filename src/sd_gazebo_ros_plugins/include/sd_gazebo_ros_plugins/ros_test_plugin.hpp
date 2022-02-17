#ifndef SD_GAZEBO_ROS_PLUGINS_ROS_TEST_HPP_
#define SD_GAZEBO_ROS_PLUGINS_ROS_TEST_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
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

  private:
	// gazebo
	gazebo::physics::ModelPtr model;
	gazebo::event::ConnectionPtr updateConnection;

	// ros
	gazebo_ros::Node::SharedPtr ros2node_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace gazebo_ros_plugins
} // namespace sd

#endif
