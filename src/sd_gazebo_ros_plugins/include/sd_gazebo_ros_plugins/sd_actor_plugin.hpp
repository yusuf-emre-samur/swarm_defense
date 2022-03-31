#ifndef SD_ACTOR_PLUGIN_HPP_
#define SD_ACTOR_PLUGIN_HPP_

// gazebo
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include <ignition/math.hh>
// ros
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
// cpp
#include <string>
#include <vector>
// interfaces
#include <sd_interfaces/msg/poses2_stamped.hpp>

namespace sd {
namespace gazebo_ros_plugins {

enum ANIMATION_ENUM { WALKING, STANDING, RUNNING };

const std::array<std::string, 3> ANIMATION_NAMES = {"walking", "standing",
													"running"};

class ActorPlugin : public gazebo::ModelPlugin
{

	/// \brief Constructor

  public:
	ActorPlugin();

	void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

  private:
	void OnUpdate(const gazebo::common::UpdateInfo& _info);

	void walkLogic(const gazebo::common::UpdateInfo& _info);

	void chooseNextTarget();

	void setAnimationType(ANIMATION_ENUM animation_type);

	void on_targets_msg_callback(
		const sd_interfaces::msg::Poses2Stamped::SharedPtr msg);
	// gazebo
	gazebo::physics::ActorPtr actor_;
	gazebo::event::ConnectionPtr update_callback_;
	gazebo::common::Time last_update_;

	// current target
	ignition::math::Vector3d target_;

	// next targets
	std::vector<ignition::math::Vector3d> next_targets_;

	// weights and vel.
	double target_weight_ = 1.0;
	double obstacle_weight_ = 1.0;
	double animation_factor_ = 1.0;
	ignition::math::Vector3d velocity_;

	// trajectory info
	gazebo::physics::TrajectoryInfoPtr trajectory_info_;

	// ros node
	gazebo_ros::Node::SharedPtr ros2node_;

	// sub
	rclcpp::Subscription<sd_interfaces::msg::Poses2Stamped>::SharedPtr
		targets_sub_;
};

} // namespace gazebo_ros_plugins
} // namespace sd
#endif
