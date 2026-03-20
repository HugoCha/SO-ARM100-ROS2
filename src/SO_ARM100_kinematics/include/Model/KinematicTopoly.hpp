#pragma once

#include "JointGroup.hpp"

#include <unordered_map>
#include <optional>

namespace SOArm100::Kinematics::Model
{
class KinematicTopology
{
public:
bool Add( JointGroup group ) {
	return groups_.insert( { group.name, group } ).second;
}

std::optional< const JointGroup > Get( const std::string& name ) const {
	auto it = groups_.find(name);
	return (it != groups_.end())
		? std::optional<const JointGroup>(it->second)
		: std::nullopt;
}

private:
std::unordered_map< std::string, JointGroup > groups_;
};
}