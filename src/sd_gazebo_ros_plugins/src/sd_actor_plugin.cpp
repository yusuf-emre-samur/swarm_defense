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
// gazebo

namespace sd {
namespace gazebo_ros_plugins {

ActorPlugin::ActorPlugin()
{
}

// when plugin is loaded
void ActorPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	this->actor_ = boost::dynamic_pointer_cast<gazebo::physics::Actor>(_model);
	this->id_ = this->actor_->GetName();
	this->ros2node_ = gazebo_ros::Node::Get();

	this->last_update_ = 0;
	this->target_ = ignition::math::Vector3d(0, 0, 0);
	// weights
	this->target_weight_ = 1.15;
	this->obstacle_weight_ = 1.5;

	// this->setAnimationType(ANIMATION_ENUM::Walking);

	this->targets_sub_ =
		ros2node_->create_subscription<sd_interfaces::msg::Position2Array>(
			"actor_targets", 1,
			std::bind(&ActorPlugin::on_position_msg_callback, this,
					  std::placeholders::_1));

	// sim callback
	this->update_callback_ = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1));

	RCLCPP_INFO(ros2node_->get_logger(), "Loaded SD Actor Plugin!");
}

// each sim step
void ActorPlugin::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
	this->walkLogic(_info);
	this->last_update_ = _info.simTime;
	auto x = this->target_;
	RCLCPP_INFO(ros2node_->get_logger(), "target");
	RCLCPP_INFO(ros2node_->get_logger(), std::to_string(x.X()).c_str());
	if ( this->next_targets_.size() > 0 ) {
		auto x = this->next_targets_.front();
		RCLCPP_INFO(ros2node_->get_logger(), "first of next");
		RCLCPP_INFO(ros2node_->get_logger(), std::to_string(x.X()).c_str());
	}
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

// set next target in vector as current target
void ActorPlugin::chooseNextTarget()
{
	if ( this->next_targets_.size() > 0 ) {
		this->setAnimationType(ANIMATION_ENUM::RUNNING);
		this->target_ = this->next_targets_.front();
		this->next_targets_.erase(this->next_targets_.begin());
	} else {
		this->setAnimationType(ANIMATION_ENUM::STANDING);
	}
}

// logic of walking
void ActorPlugin::walkLogic(const gazebo::common::UpdateInfo& _info)
{
	// Time delta
	const double dt = (_info.simTime - this->last_update_).Double();

	ignition::math::Pose3d pose = this->actor_->WorldPose();
	ignition::math::Vector3d pos = this->target_ - pose.Pos();
	ignition::math::Vector3d rpy = pose.Rot().Euler();

	const double distance = pos.Length();
	RCLCPP_INFO(ros2node_->get_logger(), "distance");
	RCLCPP_INFO(ros2node_->get_logger(), std::to_string(distance).c_str());
	// Choose a new target position if the actor has reached its current target
	if ( distance < 0.4 ) {
		RCLCPP_INFO(ros2node_->get_logger(), "under 0.4");
		this->chooseNextTarget();
		pos = this->target_ - pose.Pos();
	}

	// Normalize the direction vector, and apply the target weight
	pos = pos.Normalize() * this->target_weight_;

	// Compute the yaw orientation
	ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
	yaw.Normalize();

	// Rotate in place, instead of jumping.
	if ( std::abs(yaw.Radian()) > IGN_DTOR(10) ) {
		pose.Rot() = ignition::math::Quaterniond(
			1.5707, 0, rpy.Z() + yaw.Radian() * 0.001);
	} else {
		pose.Pos() += pos * this->velocity_ * dt;
		pose.Rot() =
			ignition::math::Quaterniond(1.5707, 0, rpy.Z() + yaw.Radian());
	}

	// Make sure the actor stays within bounds
	// pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
	// pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
	pose.Pos().Z(1.05);

	// Distance traveled is used to coordinate motion with the walking
	// animation
	const double distanceTraveled =
		(pose.Pos() - this->actor_->WorldPose().Pos()).Length();

	this->actor_->SetWorldPose(pose, false, false);
	this->actor_->SetScriptTime(this->actor_->ScriptTime() +
								(distanceTraveled * this->animation_factor_));
}

// ros
void ActorPlugin::on_position_msg_callback(
	const sd_interfaces::msg::Position2Array::SharedPtr msg)
{
	this->next_targets_.clear();
	for ( const auto& pos : msg->positions2 ) {
		auto vec3 = ignition::math::Vector3d(pos.x, pos.y, 0);
		this->next_targets_.push_back(vec3);
	}
}

GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

} // namespace gazebo_ros_plugins
} // namespace sd