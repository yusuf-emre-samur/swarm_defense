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
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
// cpp
#include <queue>
#include <string>
#include <vector>
// interfaces

namespace sd {

enum ANIMATION_ENUM { STANDING, WALKING, RUNNING, MAX };
struct WalkingPoint {
	int x, y;
	ANIMATION_ENUM walking_type;
	double wait_after = 0;
	bool target_reached = false;
};

const std::array<std::string, 3> ANIMATION_NAMES = {"standing", "walking",
													"running"};

class ActorPlugin : public gazebo::ModelPlugin
{

  public:
	ActorPlugin();

	void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

	void OnUpdate(const gazebo::common::UpdateInfo& _info);

	void walk(const gazebo::common::UpdateInfo& _info);

  private:
	void setAnimationType(ANIMATION_ENUM animation_type);

	void setNextTarget();
	void walk();

	// actor id
	std::string id_;

	// gazebo
	gazebo::physics::ActorPtr actor_;
	gazebo::event::ConnectionPtr update_callback_;
	gazebo::common::Time last_update_;
	// trajectory info
	gazebo::physics::TrajectoryInfoPtr trajectory_info_;

	// points to walk
	std::queue<WalkingPoint> points_;
	// current target
	ignition::math::Vector3d target_;
	WalkingPoint current_wp_;

	gazebo::common::Time target_reached_time_;

	// weights and vel.
	static constexpr double target_weight_ = 1.15;
	double animation_factor_ = 1.0;
	ignition::math::Vector3d velocity_;
};
} // namespace sd
#endif
