#include "Heuristic/PlanarCCDHeuristic.hpp"

#include "Global.hpp"

#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint/JointState.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/SolverHistory.hpp"
#include "Utils/Distance.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/MathUtils.hpp"

#include <limits>

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

struct PlanarCCDHeuristic::SolverBuffer
{
Mat4d local_fk;
Vec3d p_local_fk;
std::vector< Model::JointState > planar_states;
VecXd joints;
double error;
};

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

    Mat4d local_target = ComputeGroupLocalTarget( problem.seed, problem.target );
    Vec3d p_local_target = Translation( local_target );

    SolverBuffer buffer;
    Solver::SolverHistory history;
    buffer.joints = problem.seed;
    buffer.planar_states = InitializeJointStates();

    if ( IsUnreachable( p_local_target ) )
    {
        return { problem.seed, IKHeuristicState::Fail, std::numeric_limits<double>::infinity(), 0 };
    }

    for ( int iter = 0; iter < parameters_.max_iterations; iter++ )
    {
        UpdateBuffer( p_local_target, buffer.joints, buffer );
        UpdateHistory( iter, buffer, history );

        // std::cout << "=========== Iter " << iter << "===========" << std::endl; 
        // std::cout << "error = " << buffer.error << std::endl; 
        // std::cout << "Joints = \n" << buffer.joints << std::endl; 
        // std::cout << "fk = \n" << buffer.p_local_fk << std::endl; 
        if ( buffer.error < parameters_.error_tolerance )
        {
            return { 
                buffer.joints, 
                IKHeuristicState::Success, 
                buffer.error, 
                iter };
        }
        if ( history.stalled_error_cnt > parameters_.max_stalled_iterations )
        {
            return {
                history.best_joints,
                history.best_error < parameters_.error_tolerance * 10 ? IKHeuristicState::PartialSuccess : IKHeuristicState::Fail,
                history.best_error,
                iter
            };
        }

        CCD( p_local_target, buffer );
    }

    return { 
        history.best_joints, 
        IKHeuristicState::Fail, 
        history.best_error, 
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
    return distance > 1.025 * max_reach;
}

// ------------------------------------------------------------

void PlanarCCDHeuristic::CCD( const Vec3d& p_local_target, SolverBuffer& buffer ) const
{
    for ( int i = GetGroup().Size() - 1; i >= 0; i-- )
    {
        UpdateBuffer( p_local_target, buffer.joints, buffer );

        Vec3d j_to_fk  = buffer.p_local_fk - buffer.planar_states[i].Origin();
        Vec3d j_to_tgt = p_local_target - buffer.planar_states[i].Origin();

        double j_to_fk_dist  = j_to_fk.norm();
        double j_to_tgt_dist = j_to_tgt.norm();

        if ( j_to_fk_dist < 1e-6 || j_to_tgt_dist < 1e-6 )
            continue;

        double delta_angle = SignedAngle( j_to_fk, j_to_tgt, buffer.planar_states[i].GetJoint()->Axis() );

        delta_angle  = WrapAngle( delta_angle );
        buffer.planar_states[i].Value() += delta_angle;
        const auto& limits = GetChain()->GetActiveJoint( GetGroup().Index(i) )->GetLimits();
        
        buffer.planar_states[i].Value() = limits.Clamp( WrapAngle( buffer.planar_states[i].Value() ) );
        buffer.joints[GetGroup().Index(i)] = buffer.planar_states[i].Value();
    }
}

// ------------------------------------------------------------

void PlanarCCDHeuristic::UpdateBuffer( 
    const Vec3d& p_local_target, 
    const VecXd& joints, 
    SolverBuffer& buffer ) const
{
    ComputeGroupLocalJointStatesFK( 
        buffer.joints, 
        buffer.planar_states, 
        buffer.local_fk );
    
    buffer.p_local_fk = Translation( buffer.local_fk );
    buffer.error = Utils::Distance( p_local_target, buffer.p_local_fk );
}

// ------------------------------------------------------------

void PlanarCCDHeuristic::UpdateHistory( 
    int iteration, 
    const SolverBuffer& buffer, 
    Solver::SolverHistory& history ) const
{
    const int stalled_tolerance = parameters_.error_tolerance / 2.0;
    if ( buffer.error - history.last_non_stalled_error < stalled_tolerance )
    {
        history.stalled_error_cnt++;
    }
    else
    {
        history.last_non_stalled_error = buffer.error;
        history.stalled_error_cnt = 0; 
        history.last_non_stalled_error_idx = iteration;
    }

    if ( history.best_error > buffer.error )
    {
        history.best_error = buffer.error;
        history.best_joints = buffer.joints;
    }
}

// ------------------------------------------------------------

}