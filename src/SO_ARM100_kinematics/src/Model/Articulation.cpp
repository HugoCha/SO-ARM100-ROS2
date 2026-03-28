#include "Model/Articulation.hpp"

#include "Model/Joint.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

Articulation::Articulation(
	ArticulationType type,
	const std::vector< JointConstPtr >& joints,
	const Vec3d& center ) :
	type_( type ),
	joints_( joints ),
	center_( center )
{
}

// ------------------------------------------------------------

std::vector< std::shared_ptr< const Articulation > > Articulation::ExtractFromJoints(
	const std::span< const JointConstPtr >& joints )
{
	const int n_joints = joints.size();
	int index = 0;

	std::vector< std::shared_ptr< const Articulation >> articulations;

    auto last_articulation = ExtractArticulationFromLastJoints( joints );
    const int n_joints_without_last = n_joints - last_articulation->JointCount(); 

	while ( index < n_joints_without_last )
	{
		if ( index < n_joints_without_last - 2 && joints[index+2]->GetLink().GetLength() > 0 )
		{
			if ( auto articulation = ExtractArticulationFromJoints(
					 joints[index],
					 joints[index + 1],
					 joints[index + 2] ) )
			{
				articulations.push_back( std::move( articulation ) );
				index += 3;
				continue;
			}
		}
		if ( index < n_joints_without_last - 1 && joints[index+1]->GetLink().GetLength() > 0  )
		{
			if ( auto articulation = ExtractArticulationFromJoints(
					 joints[index],
					 joints[index + 1] ) )
			{
				articulations.push_back( std::move( articulation ) );
				index += 2;
				continue;
			}
		}
		if ( index < n_joints_without_last )
		{
			if ( auto articulation = ExtractArticulationFromJoints(
					 joints[index] ) )
			{
				articulations.push_back( std::move( articulation ) );
			}
			index += 1;
		}
	}

	return articulations;
}

// ------------------------------------------------------------

std::shared_ptr< const Articulation > Articulation::ExtractArticulationFromJoints(
	JointConstPtr joint1 )
{
	if ( !joint1 || joint1->IsFixed() )
		return nullptr;

	if ( joint1->IsPrismatic() )
	{
		return std::shared_ptr< Articulation >( new Articulation(
													ArticulationType::Prismatic,
													std::vector< JointConstPtr >{ joint1 },
													joint1->Origin() ) );
	}

	return std::shared_ptr< Articulation >( new Articulation(
												ArticulationType::Revolute,
												std::vector< JointConstPtr >{ joint1 },
												joint1->Origin() ) );
}

// ------------------------------------------------------------

std::shared_ptr< const Articulation > Articulation::ExtractArticulationFromJoints(
	JointConstPtr joint1,
	JointConstPtr joint2 )
{
	if ( !joint1 ||
	     !joint2 ||
	     !joint1->IsRevolute() ||
	     !joint2->IsRevolute() )
		return nullptr;

	std::vector< JointConstPtr > maybe_articulation = { joint1, joint2 };
	auto maybe_center = ComputeIntersection( maybe_articulation );

	if ( !maybe_center )
		return nullptr;

	if ( !AxesIndependent( maybe_articulation ) )
		return nullptr;

	return std::shared_ptr< Articulation >( new Articulation(
												ArticulationType::Universal,
												maybe_articulation,
												*maybe_center ) );
}

// ------------------------------------------------------------

std::shared_ptr< const Articulation > Articulation::ExtractArticulationFromJoints(
	JointConstPtr joint1,
	JointConstPtr joint2,
	JointConstPtr joint3 )
{
	if ( !joint1 || !joint2 || !joint3 ||
	     !joint1->IsRevolute() ||
	     !joint2->IsRevolute() ||
	     !joint3->IsRevolute() )
		return nullptr;

	std::vector< JointConstPtr > maybe_articulation = { joint1, joint2, joint3 };
	auto maybe_center = ComputeIntersection( maybe_articulation );

	if ( !maybe_center )
		return nullptr;

	if ( !AxesIndependent( maybe_articulation ) )
		return nullptr;

	return std::shared_ptr< Articulation >( new Articulation(
												ArticulationType::Spherical,
												maybe_articulation,
												*maybe_center ) );
}

// ------------------------------------------------------------

std::shared_ptr< const Articulation > Articulation::ExtractArticulationFromLastJoints(
    const std::span< const JointConstPtr >& joints )
{
    const int n_last_joints = std::min( 3, (int)joints.size() );
    auto maybe_last_joints = joints.last( n_last_joints );

    if ( n_last_joints == 3 )
    {
        if ( auto articulation = ExtractArticulationFromJoints(
            joints[0],
            joints[1],
            joints[2] ) )
        {
            return articulation;
        }
    }
    if ( n_last_joints >= 2 )
    {
        if ( auto articulation = ExtractArticulationFromJoints(
            joints[n_last_joints-2],
            joints[n_last_joints-1] ) )
        {
            return articulation;
        }
    }

    return ExtractArticulationFromJoints( joints[n_last_joints-1] );
}

// ------------------------------------------------------------

}