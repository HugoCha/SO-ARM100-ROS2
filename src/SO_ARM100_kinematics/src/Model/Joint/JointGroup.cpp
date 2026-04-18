#include "Model/Joint/JointGroup.hpp"

#include "Model/Joint/JointChain.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

JointGroup JointGroup::CreateFromRange(
	const std::string& name,
	int start,
	int count,
	const Mat4d& home )
{
	return { name, EnumerateIndices( start, count ), home };
}

// ------------------------------------------------------------

bool JointGroup::IsConsistent(
	const JointChain& chain,
	const JointGroup& group )
{
	const int n_joints = chain.GetActiveJointCount();
	const int n_group_joints = group.Size();

	if ( group.name.empty() ||
	     n_group_joints > n_joints ||
	     n_group_joints <= 0 ||
	     group.FirstIndex() < 0 ||
	     group.LastIndex() >= n_joints )
		return false;

	return true;
}

// ------------------------------------------------------------

bool JointGroup::IsDense( const JointGroup& group )
{
	return group.LastIndex() - group.FirstIndex() == group.Size() - 1;
}

// ------------------------------------------------------------

std::set< int > JointGroup::EnumerateIndices( int start, int count )
{
	std::set< int > range_indices;

	for ( int i = 0; i < count; i++ )
		range_indices.insert( start + i );

	return range_indices;
}

// ------------------------------------------------------------

int JointGroup::Index( int i ) const
{
	if ( i >= 0 && i < indices.size() )
	{
		auto it = indices.begin();
		std::advance( it, i );
		return *it;
	}
	return -1;
}

// ------------------------------------------------------------

VecXd JointGroup::GetGroupJoints( const VecXd& full_joints ) const {
	VecXd group_joints( indices.size() );

	for ( int i = 0; i < indices.size(); i++ )
		group_joints[i] = full_joints[Index( i )];

	return group_joints;
}

// ------------------------------------------------------------

void JointGroup::SetGroupJoints( const VecXd& group_joints, VecXd& full_joints ) const
{
	int i = 0;
	for ( int i = 0; i < indices.size(); i++ )
		full_joints[Index( i )] = group_joints[i];
}

// ------------------------------------------------------------

}