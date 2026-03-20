#pragma once

#include "Global.hpp"
#include <string>
#include <vector>

namespace SOArm100::Kinematics::Model
{
enum class JointGroupType
{
LinearBase,
ShoulderPan,
Planar2R,
PlanarNR,
Wrist,
Custom
};

struct JointGroup
{
	std::string name;
	std::vector< int > indices;
	Mat4d tip_home;
	JointGroupType type;

	static JointGroup CreateFromRange(
		const std::string& name,
		JointGroupType type,
		int start,
		int count,
		const Mat4d& home );

	VecXd GetGroupJoints( const VecXd& full_joints ) const;
	void SetGroupJoints( const VecXd& group_joints, VecXd& full_joints ) const;

	int Size() const {
		return indices.size();
	}
};
}
