#include "sd_interfaces/msg/motor_speed.hpp"
#include "sd_interfaces/msg/pose.hpp"
#include "tf2/transform_datatypes.h"
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

class DronePlugin : public gazebo::ModelPlugin
{
  public:
	/// Constructor
	DronePlugin();

	/// Destructor
	~DronePlugin()
	{
		ros2node_.reset();
	}

	void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
		ros2node_ = gazebo_ros::Node::Get(_sdf);
		RCLCPP_INFO(ros2node_->get_logger(), "In Load function");
		_model = _parent;

		if ( _sdf->HasElement("updateRate") ) {
			_rate = _sdf->Get<double>("updateRate");
		} else {
			_rate = 100.0;
		}

		if ( _sdf->HasElement("publishTf") ) {
			_publish_tf = _sdf->Get<bool>("publishTf");
		} else {
			_publish_tf = true;
		}

		if ( _sdf->HasElement("rotorThrustCoeff") ) {
			_rotor_thrust_coeff = _sdf->Get<double>("rotorThrustCoeff");
		} else {
			_rotor_thrust_coeff = 0.00025;
		}

		if ( _sdf->HasElement("rotorTorqueCoeff") ) {
			_rotor_torque_coeff = _sdf->Get<double>("rotorTorqueCoeff");
		} else {
			_rotor_torque_coeff = 0.0000074;
		}

		publisher_ =
			ros2node_->create_publisher<sd_interfaces::msg::Pose>("pose", 100);
		subscriber_ =
			ros2node_->create_subscription<sd_interfaces::msg::MotorSpeed>(
				"motor_speed_cmd", 100,
				std::bind(&DronePlugin::onMotorSpeedsMsg, this,
						  std::placeholders::_1));

		_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
			std::bind(&DronePlugin::onUpdate, this));
	}

	void onUpdate()
	{
		_pose = _model->WorldPose();
		updateThrust();
	}

	void rosThread()
	{
		publishDronePose();
	}

	void publishDronePose()
	{
		ignition::math::Pose3 pose = _pose;

		ignition::math::Vector3 rpy = pose.Rot().Euler();
		tf2::Quaternion q(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(),
						  pose.Rot().W());
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		sd_interfaces::msg::Pose pose_msg;
		pose_msg.x = pose.Pos().X();
		pose_msg.y = pose.Pos().Y();
		pose_msg.z = pose.Pos().Z();
		pose_msg.roll = roll;
		pose_msg.pitch = -pitch;
		pose_msg.yaw = yaw;
		publisher_->publish(pose_msg);

		if ( _publish_tf ) {
			tf2::Transform T;
			// T.setOrigin(
			// 	tf2::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()));
			// T.setRotation(tf2::Quaternion(pose.Rot().X(), pose.Rot().Y(),
			// 							  pose.Rot().Z(), pose.Rot().W()));
			geometry_msgs::msg::Vector3 translation;
			translation.set__x(pose.Pos().X());
			translation.set__y(pose.Pos().Y());
			translation.set__z(pose.Pos().Z());

			geometry_msgs::msg::Quaternion rotation;
			rotation.set__x(pose.Rot().X());
			rotation.set__y(pose.Rot().Y());
			rotation.set__z(pose.Rot().Z());
			rotation.set__w(pose.Rot().W());

			geometry_msgs::msg::TransformStamped transformStamped;
			transformStamped.transform.set__translation(translation);
			transformStamped.transform.set__rotation(rotation);

			transformStamped.header.stamp = ros2node_->now();
			transformStamped.header.frame_id = "world";
			transformStamped.child_frame_id = "drone";

			_tf.sendTransform(transformStamped);
		}
	}

	double calculateThrust(double w)
	{
		double thrust = _rotor_thrust_coeff * w * w;
		return thrust;
	}

	double calculateTorque(double w)
	{
		double torque = copysign(_rotor_torque_coeff * w * w, w);
		return torque;
	}

	void updateThrust()
	{
		sd_interfaces::msg::MotorSpeed cmd = _motor_speed_msg;

		int n = cmd.name.size();
		for ( int i = 0; i < n; ++i ) {
			double thrust = calculateThrust(cmd.velocity[i]);
			double torque = calculateTorque(cmd.velocity[i]);
			// ROS_INFO("torque: %f", torque);
			gazebo::physics::LinkPtr link = _model->GetLink(cmd.name[i]);
			if ( link != NULL ) {
				link->AddLinkForce(ignition::math::Vector3d(0, 0, thrust));
				link->AddRelativeTorque(ignition::math::Vector3d(0, 0, torque));
			}
		}
	}

	void onMotorSpeedsMsg(const sd_interfaces::msg::MotorSpeed::SharedPtr msg)
	{
		_motor_speed_msg = *msg;
	}

  private:
	// ros
	gazebo_ros::Node::SharedPtr ros2node_{nullptr};
	rclcpp::Publisher<sd_interfaces::msg::Pose>::SharedPtr publisher_;
	rclcpp::Subscription<sd_interfaces::msg::MotorSpeed>::SharedPtr subscriber_;
	// msg
	sd_interfaces::msg::MotorSpeed _motor_speed_msg;
	tf2_ros::TransformBroadcaster _tf;

	double _rate;
	bool _publish_tf;
	double _rotor_thrust_coeff;
	double _rotor_torque_coeff;

	gazebo::physics::ModelPtr _model;
	gazebo::event::ConnectionPtr _updateConnection;
	ignition::math::Pose3d _pose;
};

GZ_REGISTER_MODEL_PLUGIN(DronePlugin)
