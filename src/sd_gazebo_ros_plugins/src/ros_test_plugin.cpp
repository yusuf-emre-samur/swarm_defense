#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
// ros
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo {
class RosTestPlugin : public ModelPlugin
{
  public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
		// Store the pointer to the model
		this->model = _parent;

		ros2node_ = gazebo_ros::Node::Get(_sdf, _parent);

		RCLCPP_INFO(ros2node_->get_logger(), "Ros Test Plugin Load");

		// Listen to the update event. This event is broadcast every
		// simulation iteration.
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&RosTestPlugin::OnUpdate, this));
	}

	// Called by the world update start event

  public:
	void OnUpdate()
	{
		RCLCPP_INFO(ros2node_->get_logger(), "Ros Test Plugin update");
		// Apply a small linear velocity to the model.
		this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
	}

	// Pointer to the model

  private:
	physics::ModelPtr model;
	event::ConnectionPtr updateConnection;

	// ros
	gazebo_ros::Node::SharedPtr ros2node_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RosTestPlugin)
} // namespace gazebo