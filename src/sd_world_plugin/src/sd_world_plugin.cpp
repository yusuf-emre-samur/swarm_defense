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

	this->ros2node_ = gazebo_ros::Node::Get(_sdf);
	const gazebo_ros::QoS& qos = this->ros2node_->get_qos();

	// object to track
	auto object = _sdf->GetElement("objects")->GetElement("object");
	while ( object ) {
		auto name = object->Get<std::string>();
		auto model_ptr = this->world_->ModelByName(name);
		if ( !model_ptr ) {
			RCLCPP_ERROR(ros2node_->get_logger(),
						 std::string("Model with name: " + name +
									 " not found in world !")
							 .c_str());
		}
		track_objects_.push_back(name);
		object = object->GetNextElement("object");
	}

	// publisher
	this->publisher_ =
		ros2node_->create_publisher<sd_interfaces::msg::WorldObjects>(
			"objects", qos.get_publisher_qos("objects", rclcpp::QoS(10)));

	this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
		std::bind(&WorldPlugin::OnUpdate, this));

	RCLCPP_INFO(ros2node_->get_logger(), "Loaded ROS WorldPlugin !");
}

void WorldPlugin::OnUpdate()
{
	sd_interfaces::msg::WorldObjects object_list;
	for ( const auto& object : this->track_objects_ ) {
		auto model = this->world_->ModelByName(object);
		if ( model ) {
			auto bbox = model->BoundingBox();

			sd_interfaces::msg::Object obj;
			obj.id = object;
			// center
			obj.bbox.center.x = model->WorldPose().Pos().X();
			obj.bbox.center.y = model->WorldPose().Pos().Y();
			obj.bbox.center.z = model->WorldPose().Pos().Z();
			// min
			obj.bbox.pmin.x = bbox.Min().X();
			obj.bbox.pmin.y = bbox.Min().Y();
			obj.bbox.pmin.z = bbox.Min().Z();
			// max
			obj.bbox.pmax.x = bbox.Max().X();
			obj.bbox.pmax.y = bbox.Max().Y();
			obj.bbox.pmax.z = bbox.Max().Z();

			object_list.objects.push_back(obj);
		} else {
			RCLCPP_ERROR_ONCE(
				this->ros2node_->get_logger(),
				std::string("Model " + object + " not found!").c_str());
		}
	}
	// publish
	object_list.header.frame_id = "world";
	object_list.header.stamp = this->ros2node_->now();
	this->publisher_->publish(object_list);
}

GZ_REGISTER_WORLD_PLUGIN(WorldPlugin)
} // namespace sd