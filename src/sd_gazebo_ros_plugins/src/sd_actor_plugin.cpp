/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

// interface
#include "sd_gazebo_ros_plugins/sd_actor_plugin.hpp"
// other
#include <functional>
#include <thread>

namespace sd {
namespace gazebo_ros_plugins {

constexpr double actor_roll = 1.5707;

ActorPlugin::ActorPlugin()
{
}

// when plugin is loaded
void ActorPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	this->actor_ = boost::dynamic_pointer_cast<gazebo::physics::Actor>(_model);
	this->id_ = this->actor_->GetName();
	// ros
	this->ros2node_ = gazebo_ros::Node::Get(_sdf, _model);

	this->action_server_ =
		rclcpp_action::create_server<sd_interfaces::action::Walk>(
			this->ros2node_, "walk",
			std::bind(&ActorPlugin::handle_goal, this, std::placeholders::_1,
					  std::placeholders::_2),
			std::bind(&ActorPlugin::handle_cancel, this, std::placeholders::_1),
			std::bind(&ActorPlugin::handle_accepted, this,
					  std::placeholders::_1));

	// params
	this->last_update_ = 0;
	this->target_ = this->actor_->WorldPose().Pos();
	// default animation type is standing

	this->setAnimationType(ANIMATION_ENUM::STANDING);
	// this->chooseNextTarget();

	// sim callback
	this->update_callback_ = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1));

	RCLCPP_INFO(
		ros2node_->get_logger(),
		std::string("Loaded SD Actor Plugin for Actor ID: " + this->id_ + " !")
			.c_str());
}

// action goal request
rclcpp_action::GoalResponse ActorPlugin::handle_goal(
	const rclcpp_action::GoalUUID& uuid,
	std::shared_ptr<const sd_interfaces::action::Walk::Goal> goal)
{
	RCLCPP_INFO(this->ros2node_->get_logger(), "Received walk goal!");
	// check if action already exists
	if ( this->action_set_ ) {
		RCLCPP_INFO(this->ros2node_->get_logger(),
					"Error: Already in walk action!");
		(void)uuid;
		return rclcpp_action::GoalResponse::REJECT;
	} else {
		RCLCPP_INFO(this->ros2node_->get_logger(), "Starting walk action!");
		this->action_set_ = true;
		(void)uuid;
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}
}

// action cancel request
rclcpp_action::CancelResponse ActorPlugin::handle_cancel(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<sd_interfaces::action::Walk>>
		goal_handle)
{
	(void)goal_handle;
	this->setAnimationType(ANIMATION_ENUM::STANDING);
	RCLCPP_INFO(this->ros2node_->get_logger(), "Canceled walk action!");
	this->action_set_ = false;
	return rclcpp_action::CancelResponse::ACCEPT;
}

// action accepted
void ActorPlugin::handle_accepted(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<sd_interfaces::action::Walk>>
		goal_handle)
{
	// this needs to return quickly to avoid blocking the executor, so spin up a
	// new thread
	std::thread{std::bind(&ActorPlugin::execute, this, std::placeholders::_1,
						  this->ros2node_->get_clock()),
				goal_handle}
		.detach();
}

void ActorPlugin::execute(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<sd_interfaces::action::Walk>>
		goal_handle,
	rclcpp::Clock::SharedPtr rosclock)
{
	RCLCPP_INFO(this->ros2node_->get_logger(), "Start walking to goal!");
	rclcpp::Rate loop_rate(1000);

	auto last_time = rosclock->now();
	// goal, feedback, result
	auto goal = goal_handle->get_goal();
	auto feedback = std::make_shared<sd_interfaces::action::Walk::Feedback>();
	auto result = std::make_shared<sd_interfaces::action::Walk::Result>();
	// set goal target
	this->target_ = ignition::math::Vector3d(goal->goal_x, goal->goal_y, 0);

	// set walking animation type
	if ( goal->walking_type < ANIMATION_ENUM::MAX && goal->walking_type > 0 ) {
		this->setAnimationType(static_cast<ANIMATION_ENUM>(goal->walking_type));
	} else {
		RCLCPP_INFO(this->ros2node_->get_logger(),
					"Error: False walking type, using default walking!");
		this->setAnimationType(ANIMATION_ENUM::WALKING);
	}

	auto paused = false;
	std::chrono::nanoseconds paused_time(0);

	while ( rclcpp::ok() ) {
		// dt of rosclock is too slow, buggy walking animation
		// sum dt and check if reached some treshold of 500ms
		auto dt = rosclock->now() == last_time;
		last_time = rosclock->now();
		if ( dt ) {
			paused_time += loop_rate.period();
		} else {
			paused_time = std::chrono::nanoseconds(0);
			paused = false;
		}
		if ( paused_time > std::chrono::milliseconds(500) ) {
			paused = true;
		}

		if ( !paused ) {

			auto pose = this->actor_->WorldPose();
			feedback->current_x = pose.Pos().X();
			feedback->current_y = pose.Pos().Y();

			auto pos_dif = this->target_ - pose.Pos();
			auto rpy = pose.Rot().Euler();

			// target reached when dist < 0.1
			const auto distance =
				std::sqrt(std::pow(this->target_.X() - pose.Pos().X(), 2) +
						  std::pow(this->target_.Y() - pose.Pos().Y(), 2));
			if ( distance < 0.1 ) {
				this->setAnimationType(ANIMATION_ENUM::STANDING);
				break;
			}

			// normalize the direction vector, and apply the target weight
			pos_dif = pos_dif.Normalize() * this->target_weight_;

			// compute the yaw orientation
			ignition::math::Angle yaw =
				atan2(pos_dif.Y(), pos_dif.X()) + actor_roll - rpy.Z();
			yaw.Normalize();

			// rotate in place, instead of jumping.
			if ( std::abs(yaw.Radian()) > IGN_DTOR(10) ) {
				pose.Rot() = ignition::math::Quaterniond(
					actor_roll, 0, rpy.Z() + yaw.Radian() * 0.01);
			}
			// move
			else {
				pose.Pos() += pos_dif * this->velocity_ * 0.001;
				pose.Rot() = ignition::math::Quaterniond(
					actor_roll, 0, rpy.Z() + yaw.Radian());
			}

			pose.Pos().Z(1.05);

			// distance traveled is used to coordinate motion with the
			// walking animation
			const double distanceTraveled =
				(pose.Pos() - this->actor_->WorldPose().Pos()).Length();

			this->actor_->SetWorldPose(pose, false, false);
			this->actor_->SetScriptTime(
				this->actor_->ScriptTime() +
				(distanceTraveled * this->animation_factor_));

			// Publish feedback
			goal_handle->publish_feedback(feedback);
		}
		// Check if there is a cancel request
		if ( goal_handle->is_canceling() ) {
			result->success = false;
			goal_handle->canceled(result);
			this->setAnimationType(ANIMATION_ENUM::STANDING);
			return;
		}

		loop_rate.sleep();
	}

	// Check if goal is done
	if ( rclcpp::ok() ) {
		result->success = true;
		goal_handle->succeed(result);
		this->action_set_ = false;
		RCLCPP_INFO(this->ros2node_->get_logger(), "Goal succeeded !");
	}
}
// each sim step
void ActorPlugin::OnUpdate(const gazebo::common::UpdateInfo& _info)
{

	if ( !this->action_set_ ) {
		// make actor stand
		auto pose = this->actor_->WorldPose();
		auto rpy = ignition::math::Vector3d(actor_roll, 0, 0);
		auto q = ignition::math::Quaterniond::EulerToQuaternion(rpy);
		pose.Rot() = q;
		this->actor_->SetWorldPose(pose);
	}
	this->last_update_ = _info.simTime;
}

// set animation type, e.g. walking, running & standing
void ActorPlugin::setAnimationType(ANIMATION_ENUM animation_type)
{
	auto skel_anims = this->actor_->SkeletonAnimations();
	if ( skel_anims.find(ANIMATION_NAMES[animation_type]) ==
		 skel_anims.end() ) {
		gzerr << "Skeleton animation " << ANIMATION_NAMES[animation_type]
			  << " not found.\n";
	} else {
		// set custom trajectory
		this->trajectory_info_.reset(new gazebo::physics::TrajectoryInfo());
		this->trajectory_info_->type = ANIMATION_NAMES[animation_type];
		this->trajectory_info_->duration = 1.0;

		this->actor_->SetCustomTrajectory(this->trajectory_info_);
	}
	switch ( animation_type ) {
	case ANIMATION_ENUM::WALKING:
		this->velocity_ = 0.8;
		this->animation_factor_ = 4.5;
		break;
	case ANIMATION_ENUM::STANDING:
		this->velocity_ = 0;
		this->animation_factor_ = 4.5;
		break;
	case ANIMATION_ENUM::RUNNING:
		this->velocity_ = 3;
		this->animation_factor_ = 1.5;
		break;
	default:
		this->velocity_ = 0.8;
		break;
	}
}

GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

} // namespace gazebo_ros_plugins
} // namespace sd