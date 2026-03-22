#pragma once

#include "Global.hpp"

#include <string>
#include <set>

namespace SOArm100::Kinematics::Model
{
constexpr std::string revolute_base_name = "revolute_base";
constexpr std::string prismatic_base_name = "prismatic_base";
constexpr std::string planarNR_name = "planar_NR";
constexpr std::string wrist_name = "wrist";

class JointChain;

struct JointGroup
{
	std::string name;
	std::set< int > indices;
	Mat4d tip_home;

	static JointGroup CreateFromRange(
		const std::string& name,
		int start,
		int count,
		const Mat4d& home );

	static bool IsConsistent( 
		const JointChain& chain, 
		const JointGroup& group );

	static bool IsDense( const JointGroup& group );

	VecXd GetGroupJoints( const VecXd& full_joints ) const;
	void SetGroupJoints( const VecXd& group_joints, VecXd& full_joints ) const;
	
	int FirstIndex() const {
		return !indices.empty() ? *indices.begin() : -1;
	}

	int Index( int i ) const;

	int LastIndex() const {
		return !indices.empty() ? *std::prev( indices.end() ) : -1;
	}

	int Size() const {
		return indices.size();
	}

protected:
	static std::set< int > EnumerateIndices( int start, int count );
};

struct RevoluteBaseJointGroup : public JointGroup
{
	RevoluteBaseJointGroup( const Mat4d& home ) :
		JointGroup( revolute_base_name, { 0 }, home )
	{
	}
};

struct PrismaticBaseJointGroup : public JointGroup
{
	PrismaticBaseJointGroup( const Mat4d& home ) :
		JointGroup( prismatic_base_name, { 0 }, home )
	{
	}
};

struct PlanarNRJointGroup : public JointGroup
{
	PlanarNRJointGroup( int start, int count, const Mat4d& home ) :
		JointGroup( planarNR_name, EnumerateIndices( start, count ), home )
	{
	}

	PlanarNRJointGroup( int sub_planar_index, int start, int count, const Mat4d& home ) :
		JointGroup( planarNR_name + std::to_string( sub_planar_index ), EnumerateIndices( start, count ), home )
	{
	}
};

struct WristJointGroup : public JointGroup
{
	WristJointGroup( int start, int count, const Mat4d& home ) :
		JointGroup( wrist_name, EnumerateIndices( start, count ), home )
	{
	}
};

}
