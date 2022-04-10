#ifndef SD_ACTOR_PLUGIN_HPP_
#define SD_ACTOR_PLUGIN_HPP_

// gazebo
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include <ignition/math.hh>
// ros
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
// cpp
#include <string>
#include <vector>
// interfaces
#include <sd_interfaces/action/walk.hpp>
#include <sd_interfaces/msg/position2_array.hpp>
#include <sd_interfaces/msg/walking_type.hpp>

namespace sd {
namespace gazebo_ros_plugins {

enum ANIMATION_ENUM { STANDING, WALKING, RUNNING, MAX };

const std::array<std::string, 3> ANIMATION_NAMES = {"standing", "walking",
													"running"};

class ActorPlugin : public gazebo::ModelPlugin
{

  public:
	ActorPlugin();

	void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

  private:
	void OnUpdate(const gazebo::common::UpdateInfo& _info);

	void walkLogic(const gazebo::common::UpdateInfo& _info);

	void chooseNextTarget();

	void setAnimationType(ANIMATION_ENUM animation_type);

	void
	on_position_msg(const sd_interfaces::msg::Position2Array::SharedPtr msg);

	void
	on_walking_type_msg(const sd_interfaces::msg::WalkingType::SharedPtr msg);

	std::string id_;

	// gazebo
	gazebo::physics::ActorPtr actor_;
	gazebo::event::ConnectionPtr update_callback_;
	gazebo::common::Time last_update_;

	// current target
	ignition::math::Vector3d target_;

	// weights and vel.
	static constexpr double target_weight_ = 1.15;
	double animation_factor_ = 1.0;
	ignition::math::Vector3d velocity_;

	// trajectory info
	gazebo::physics::TrajectoryInfoPtr trajectory_info_;

	// ros node
	gazebo_ros::Node::SharedPtr ros2node_;

	// ros walk action server
	rclcpp_action::Server<sd_interfaces::action::Walk>::SharedPtr
		action_server_;

	// goal handel
	rclcpp_action::GoalResponse
	handle_goal(const rclcpp_action::GoalUUID& uuid,
				std::shared_ptr<const sd_interfaces::action::Walk::Goal> goal);
	// cancel handle
	rclcpp_action::CancelResponse
	handle_cancel(const std::shared_ptr<
				  rclcpp_action::ServerGoalHandle<sd_interfaces::action::Walk>>
					  goal_handle);

	// accepted handle
	void handle_accepted(
		const std::shared_ptr<
			rclcpp_action::ServerGoalHandle<sd_interfaces::action::Walk>>
			goal_handle);

	// execute thread
	void
	execute(const std::shared_ptr<
				rclcpp_action::ServerGoalHandle<sd_interfaces::action::Walk>>
				goal_handle,
			rclcpp::Clock::SharedPtr rosclock);

	bool action_set_ = false;
};

} // namespace gazebo_ros_plugins
} // namespace sd
#endif
