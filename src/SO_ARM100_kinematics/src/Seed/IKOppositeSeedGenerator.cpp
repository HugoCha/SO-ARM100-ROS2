#include "Seed/IKOppositeSeedGenerator.hpp"

#include "Solver/IKProblem.hpp" 

#include <unordered_set>
#include <vector>

namespace SOArm100::Kinematics::Seed
{

// ------------------------------------------------------------

IKOppositeSeedGenerator::IKOppositeSeedGenerator( 
    Model::KinematicModelConstPtr model,
    const std::span<const int>& joint_indexes ) :
    model_(model),
    joint_indexes_( UniqueValidJointIndexes( model, joint_indexes ) )
{}

// ------------------------------------------------------------

IKOppositeSeedGenerator::IKOppositeSeedGenerator( Model::KinematicModelConstPtr model ) :
    model_(model),
    joint_indexes_( EnumerateAllJointIndexes(model) )
{}

// ------------------------------------------------------------

VecXd IKOppositeSeedGenerator::Generate( const Solver::IKProblem& problem ) const
{
    VecXd seed = problem.seed;

    for ( int i = 0; i < joint_indexes_.size(); i++ )
    {
        int joint_index = joint_indexes_[i];
        auto limits = model_->GetChain()->GetActiveJointLimits( joint_index ); 
        seed[joint_index] = limits.Center() - seed[joint_index];
    }

    return seed;
}

// ------------------------------------------------------------

std::vector<int> IKOppositeSeedGenerator::EnumerateAllJointIndexes( 
    Model::KinematicModelConstPtr model )
{
    const int n_joints = model->GetChain()->GetActiveJointCount();
    std::vector<int> joint_indexes( n_joints );
    for ( int i = 0; i < n_joints; i++ )
        joint_indexes[i] = i;
    return joint_indexes;
}

// ------------------------------------------------------------

std::vector<int> IKOppositeSeedGenerator::UniqueValidJointIndexes( 
    Model::KinematicModelConstPtr model, 
    const std::span<const int> joint_indexes )
{
    std::unordered_set<int> joint_indexes_set;
    const int max_index = model->GetChain()->GetActiveJointCount() - 1;

    for ( int i=0; i < joint_indexes.size(); i++ )
        if ( joint_indexes[i] >= 0 && joint_indexes[i] <= max_index )
            joint_indexes_set.insert( joint_indexes[i] );

    return std::vector< int >( joint_indexes_set.begin(), joint_indexes_set.end() );
}

// ------------------------------------------------------------

}