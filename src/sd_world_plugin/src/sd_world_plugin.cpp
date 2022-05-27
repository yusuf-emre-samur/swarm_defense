#include <sd_world_plugin/sd_world_plugin.hpp>

namespace sd {

WorldPlugin::WorldPlugin() : gazebo::WorldPlugin()
{
}

WorldPlugin::~WorldPlugin()
{
	ros2node_.reset();
}

void WorldPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
	this->world_ = _world;

	ros2node_ = gazebo_ros::Node::Get(_sdf);

	// actor names
	auto actor = _sdf->GetElement("actors")->GetElement("actor");
	while ( actor ) {
		auto name = actor->Get<std::string>();
		track_objects_.push_back(name);
		actor = actor->GetNextElement("actor");
	}

	// drone names
	auto drone = _sdf->GetElement("drones")->GetElement("drone");
	while ( drone ) {
		auto name = drone->Get<std::string>();
		track_objects_.push_back(name);
		drone = drone->GetNextElement("drone");
	}

	// publisher
	publisher_ = ros2node_->create_publisher<sd_interfaces::msg::ObjectsVector>(
		"world_objects", 10);

	this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&WorldPlugin::OnUpdate, this));

	RCLCPP_INFO(ros2node_->get_logger(), "Loaded ROS WorldPlugin !");
}

void WorldPlugin::OnUpdate()
{
	// for ( const auto& object : this->track_objects_ ) {
	// 	auto model = this->world_->ModelByName(object);
	// 	RCLCPP_INFO(ros2node_->get_logger(), model->GetName().c_str());
	// }
}

GZ_REGISTER_WORLD_PLUGIN(WorldPlugin)
} // namespace sd