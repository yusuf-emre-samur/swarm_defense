#ifndef SD_UTILS_
#define SD_UTILS_
#include <eigen3/Eigen/Dense>
#include <sd_interfaces/msg/position.hpp>

sd_interfaces::msg::Position eigen_to_sd_pos(const Eigen::Vector3d& pos)
{
	sd_interfaces::msg::Position p;
	p.x = pos.x();
	p.y = pos.y();
	p.z = pos.z();
	return p;
}

Eigen::Vector3d sd_pos_to_eigen(const sd_interfaces::msg::Position& pos)
{
	Eigen::Vector3d p;
	p.x() = pos.x;
	p.y() = pos.y;
	p.z() = pos.z;
	return p;
}

#endif