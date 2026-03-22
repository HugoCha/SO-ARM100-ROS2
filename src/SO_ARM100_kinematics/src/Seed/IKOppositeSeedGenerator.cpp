#include "Seed/IKOppositeSeedGenerator.hpp"

#include "Solver/IKProblem.hpp"

namespace SOArm100::Kinematics::Seed
{

// ------------------------------------------------------------

IKOppositeSeedGenerator::IKOppositeSeedGenerator(
	Model::KinematicModelConstPtr model,
	const std::set< int >& joint_indexes ) :
	model_( model ),
	joint_indexes_( joint_indexes )
{
}

// ------------------------------------------------------------

IKOppositeSeedGenerator::IKOppositeSeedGenerator( 
	Model::KinematicModelConstPtr model, 
	Model::JointGroup group ) :
	IKOppositeSeedGenerator( model, group.indices )
{
}

// ------------------------------------------------------------

IKOppositeSeedGenerator::IKOppositeSeedGenerator( Model::KinematicModelConstPtr model ) :
	model_( model ),
	joint_indexes_( EnumerateAllJointIndexes( model ) )
{
}

// ------------------------------------------------------------

VecXd IKOppositeSeedGenerator::Generate( const Solver::IKProblem& problem ) const
{
	VecXd seed = problem.seed;

	int i = 0;
	for ( auto it = joint_indexes_.begin(); it != joint_indexes_.end(); ++it )
	{
		int joint_index = *it;
		auto limits = model_->GetChain()->GetActiveJointLimits( joint_index );
		seed[joint_index] = limits.Center() - seed[joint_index];
		i++;
	}

	return seed;
}

// ------------------------------------------------------------

std::set< int > IKOppositeSeedGenerator::EnumerateAllJointIndexes(
	Model::KinematicModelConstPtr model )
{
	const int n_joints = model->GetChain()->GetActiveJointCount();
	std::set< int > joint_indexes;
	for ( int i = 0; i < n_joints; i++ )
		joint_indexes.insert( i );
	return joint_indexes;
}

// ------------------------------------------------------------

}