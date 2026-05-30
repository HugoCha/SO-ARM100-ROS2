#include "Heuristic/PlanarCCDHeuristic.hpp"

#include "Global.hpp"

#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint/JointState.hpp"
#include "Solver/IKProblem.hpp"
#include "Utils/Distance.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/MathUtils.hpp"
#include <limits>

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

PlanarCCDHeuristic::PlanarCCDHeuristic(
	Model::KinematicModelConstPtr model,
	Model::JointGroup planar_group,
	SolverParameters parameters ) :
    Model::IKJointGroupModelBase( model, planar_group ),
    parameters_( parameters )
{
}

// ------------------------------------------------------------

IKPresolution PlanarCCDHeuristic::Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const
{
    if ( model_->IsUnreachable( problem.target ) )
	{
        return { problem.seed, IKHeuristicState::Fail, std::numeric_limits<double>::infinity(), 0 };
	}
    
    const int n_joints = model_->GetChain()->GetActiveJointCount();
	if ( problem.seed.size() != n_joints )
	{
		return { problem.seed, IKHeuristicState::Fail, std::numeric_limits<double>::infinity(), 0 };
	}

    VecXd joints = problem.seed;
    auto planar_states = InitializeJointStates();
    double error;
    Mat4d local_fk;
    Vec3d p_local_fk;
    Mat4d local_target = ComputeGroupLocalTarget( problem.seed, problem.target );
    Vec3d p_local_target = Translation( local_target );

    if ( IsUnreachable( p_local_target ) )
    {
        return { problem.seed, IKHeuristicState::Fail, std::numeric_limits<double>::infinity(), 0 };
    }

    for ( int iter = 0; iter < parameters_.max_iterations; iter++ )
    {
        ComputeGroupLocalJointStatesFK( 
            joints, 
            planar_states, 
            local_fk );
        
        p_local_fk = Translation( local_fk );
        error =  Utils::Distance( p_local_fk, p_local_target );

        if ( error < parameters_.error_tolerance )
        {
            return { 
                joints, 
                IKHeuristicState::Success, 
                error, 
                iter };
        }

        for ( int i = GetGroup().Size() - 1; i >= 0; i-- )
        {
            ComputeGroupLocalJointStatesFK( 
                joints, 
                planar_states, 
                local_fk );

            p_local_fk = Translation( local_fk );

            Vec3d j_to_fk  = p_local_fk - planar_states[i].Origin();
            Vec3d j_to_tgt = p_local_target - planar_states[i].Origin();

            double j_to_fk_dist  = j_to_fk.norm();
            double j_to_tgt_dist = j_to_tgt.norm();

            if ( j_to_fk_dist < 1e-6 || j_to_tgt_dist < 1e-6 )
                continue;

            double delta_angle = SignedAngle( j_to_fk, j_to_tgt, planar_states[i].Axis() );

            delta_angle  = WrapAngle( delta_angle );
            planar_states[i].Value() += delta_angle;
            const auto& limits = GetChain()->GetActiveJoint( GetGroup().Index(i) )->GetLimits();
            
            planar_states[i].Value() = limits.Clamp( planar_states[i].Value() );
            joints[GetGroup().Index(i)] = planar_states[i].Value();
        }
    }

    return { 
        joints, 
        IKHeuristicState::PartialSuccess, 
        error, 
        parameters_.max_iterations };
}

// ------------------------------------------------------------

std::vector< Model::JointState > PlanarCCDHeuristic::InitializeJointStates() const 
{
    std::vector< Model::JointState > planar_states;
    for ( int i = 0; i < GetGroup().Size(); i++ )
    {
        auto joint_idx = GetGroup().Index( i );
        const auto& joint = GetChain()->GetActiveJoint( joint_idx );
        planar_states.emplace_back( Model::JointState( joint ) );
    }
    return planar_states;
}

// ------------------------------------------------------------

bool PlanarCCDHeuristic::IsUnreachable( const Vec3d& p_local_target ) const
{
    const auto& first_joint = GetChain()->GetActiveJoint( GetGroup().FirstIndex() );
    double distance = Utils::Distance( first_joint->Origin(), p_local_target );
    double max_reach = GroupMaxReach();
    return distance > 1.01 * max_reach;
}

// ------------------------------------------------------------

}