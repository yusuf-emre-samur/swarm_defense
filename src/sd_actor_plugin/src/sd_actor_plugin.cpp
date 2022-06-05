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
#include "sd_actor_plugin/sd_actor_plugin.hpp"
// other
#include <functional>
#include <thread>

namespace sd {

constexpr double actor_roll = 1.5707;

ActorPlugin::ActorPlugin()
{
}

// when plugin is loaded
void ActorPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	this->actor_ = boost::dynamic_pointer_cast<gazebo::physics::Actor>(_model);
	this->id_ = this->actor_->GetName();

	// object to track
	if ( _sdf->HasElement("points") ) {
		auto points = _sdf->GetElement("points");
		if ( points->HasElement("point") ) {
			auto point = points->GetElement("point");
			while ( point ) {
				WalkingPoint wp;
				if ( point->HasAttribute("type") ) {
					auto walking_type =
						point->GetAttribute("type")->GetAsString();
					if ( walking_type == "run" ) {
						wp.walking_type = ANIMATION_ENUM::RUNNING;
					}
					if ( walking_type == "walk" ) {
						wp.walking_type = ANIMATION_ENUM::WALKING;
					}
				} else {
					wp.walking_type = ANIMATION_ENUM::WALKING;
				}
				if ( point->HasAttribute("wait_after") ) {
					auto wait_after =
						point->GetAttribute("wait_after")->GetAsString();
					wp.wait_after = std::stod(wait_after);
				}
				auto p = point->Get<ignition::math::Vector2d>();
				wp.x = p.X();
				wp.y = p.Y();
				this->points_.push(wp);
				point = point->GetNextElement("point");
			}
		}
	}

	this->setNextTarget();
	// params
	this->last_update_ = 0;

	// sim callback
	this->update_callback_ = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1));
}

// each sim step
void ActorPlugin::OnUpdate(const gazebo::common::UpdateInfo& _info)
{
	if ( this->current_wp_.target_reached ) {
		// if wait is over
		if ( (_info.simTime - this->target_reached_time_).sec >
			 this->current_wp_.wait_after ) {
			this->setNextTarget();
		}
	} else {
		this->walk(_info);
	}
	this->last_update_ = _info.simTime;
}

void ActorPlugin::setNextTarget()
{
	if ( this->points_.size() > 0 ) {
		this->current_wp_ = this->points_.front();
		this->points_.pop();
		this->setAnimationType(this->current_wp_.walking_type);
		this->target_.X() = this->current_wp_.x;
		this->target_.Y() = this->current_wp_.y;
	} else {
		this->target_ = this->actor_->WorldPose().Pos();
		this->setAnimationType(ANIMATION_ENUM::STANDING);
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

void ActorPlugin::walk(const gazebo::common::UpdateInfo& _info)
{
	auto pose = this->actor_->WorldPose();

	auto pos_dif = this->target_ - pose.Pos();
	auto rpy = pose.Rot().Euler();

	// target reached when dist < 0.1
	const auto distance =
		std::sqrt(std::pow(this->target_.X() - pose.Pos().X(), 2) +
				  std::pow(this->target_.Y() - pose.Pos().Y(), 2));

	// target reached
	if ( distance < 0.1 ) {
		this->target_reached_time_ = _info.simTime;
		this->current_wp_.target_reached = true;
		this->setAnimationType(ANIMATION_ENUM::STANDING);
	}

	// normalize the direction vector, and apply the target weight
	pos_dif = pos_dif.Normalize() * this->target_weight_;

	// compute the yaw orientation
	ignition::math::Angle yaw =
		atan2(pos_dif.Y(), pos_dif.X()) + actor_roll - rpy.Z();
	yaw.Normalize();

	// rotate in place, instead of jumping.
	if ( std::abs(yaw.Radian()) > IGN_DTOR(10) ) {
		pose.Rot() = ignition::math::Quaterniond(actor_roll, 0,
												 rpy.Z() + yaw.Radian() * 0.1);
	}
	// move
	else {
		pose.Pos() += pos_dif * this->velocity_ * 0.01;
		pose.Rot() =
			ignition::math::Quaterniond(actor_roll, 0, rpy.Z() + yaw.Radian());
	}

	pose.Pos().Z(1.05);

	// distance traveled is used to coordinate motion with the
	// walking animation
	const double distanceTraveled =
		(pose.Pos() - this->actor_->WorldPose().Pos()).Length();

	this->actor_->SetWorldPose(pose, false, false);
	this->actor_->SetScriptTime(this->actor_->ScriptTime() +
								(distanceTraveled * this->animation_factor_));
}

GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

} // namespace sd