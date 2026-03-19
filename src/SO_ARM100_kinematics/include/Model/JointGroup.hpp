#pragma once

#include "Global.hpp"
#include <string>
#include <vector>

namespace SOArm100::Kinematics::Model
{
struct JointGroup
{
	std::string name;
	std::vector< int > indices;
	Mat4d tip_home;

	static JointGroup CreateFromRange(
		const std::string& name,
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
