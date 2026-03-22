#include "Heuristic/Planar2RHeuristic.hpp"

#include "Global.hpp"

#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Heuristic/JointGroupHeuristic.hpp"
#include "Model/JointGroup.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <cmath>

namespace SOArm100::Kinematics::Heuristic
{

// ------------------------------------------------------------

Planar2RHeuristic::Planar2RHeuristic( 
    Model::KinematicModelConstPtr model, 
    Model::PlanarNRJointGroup planar_group ) :
    JointGroupHeuristic( model, planar_group ),
    reference_direction_( ComputeReferenceDirection( model, planar_group ) ),
    up_direction_( ComputeUpDirection( reference_direction_, model, planar_group ) )
{}
    
// ------------------------------------------------------------

Vec3d Planar2RHeuristic::ComputeReferenceDirection(
    Model::KinematicModelConstPtr model,
	const Model::JointGroup planar_group )
{
    Vec3d p_shoulder = model->GetChain()->GetActiveJoint( planar_group.FirstIndex() )->Origin();
    Vec3d p_elbow    = model->GetChain()->GetActiveJoint( planar_group.Index( 1 ) )->Origin();
    return ( p_elbow - p_shoulder ).normalized();
}

// ------------------------------------------------------------

Vec3d Planar2RHeuristic::ComputeUpDirection(
    const Vec3d& reference_direction,
    Model::KinematicModelConstPtr model,
    const Model::JointGroup planar_group )
{
    Vec3d plane_normal = model->GetChain()->GetActiveJoint( planar_group.FirstIndex())->Axis();
    return plane_normal.cross( reference_direction );
}

// ------------------------------------------------------------

double Planar2RHeuristic::L1() const
{
    return GetChain()->GetActiveJointLink( GetGroup().FirstIndex() ).GetLength();
}

// ------------------------------------------------------------

double Planar2RHeuristic::L2() const
{
    return GetChain()->GetActiveJointLink( GetGroup().Index( 1 ) ).GetLength();
}

// ------------------------------------------------------------

IKPresolution Planar2RHeuristic::Presolve( 
    const Solver::IKProblem& problem, 
    const Solver::IKRunContext& context ) const
{
    VecXd seed = problem.seed;
    VecXd planar_solution( 2 );

    auto T_first_planar = ComputeGroupFirstJointPose( seed );
    auto T_group_target = ComputeGroupTarget( seed, problem.target );

    Vec3d p_first_planar = Translation( T_first_planar );
    Vec3d p_group_target = Translation( T_group_target );
    Vec3d wrist_dir = p_group_target - p_first_planar;

    double x_local = wrist_dir.dot( reference_direction_ );
    double y_local = wrist_dir.dot( up_direction_ );
    double D_squared = x_local * x_local + y_local * y_local;
    double D  = sqrt( D_squared );
    double L1 = Planar2RHeuristic::L1();
    double L2 = Planar2RHeuristic::L2();

    // Unreachable
    if ( D > L1 + L2 || D < std::abs( L1 - L2 ) )
        return { {}, IKHeuristicState::Fail };

    VecXd elbow_up_solution( 2 );
    VecXd elbow_down_solution( 2 );

    // Elbow
    double cos_beta = ( L1 * L1 + L2 * L2 - D_squared ) / ( 2.0 * L1 * L2 );
    double beta = std::acos( cos_beta );
    double elbow_up = M_PI - beta;
    double elbow_down = -elbow_up;

    // Soulder
    double alpha_target  = std::atan2( y_local, x_local );
    double cos_alpha_int = ( L1 * L1 + D_squared - L2 * L2) / ( 2.0 * L1 * D ); 
    double alpha_int = std::acos( cos_alpha_int );
    double shoulder_elbow_up = alpha_target - alpha_int;
    double shoulder_elbow_down = alpha_target + alpha_int;
    
    elbow_up_solution[0] = shoulder_elbow_up;
    elbow_up_solution[1] = elbow_up;

    elbow_down_solution[0] = shoulder_elbow_down;
    elbow_down_solution[1] = elbow_down;

    VecXd planar_seed = GetGroup().GetGroupJoints( problem.seed );
    if ( !ValidateAndSelectElbowConfiguration( planar_seed, elbow_up_solution, elbow_down_solution, planar_solution ) )
    {
        return { seed, IKHeuristicState::PartialSuccess };
    }

    GetGroup().SetGroupJoints( planar_solution, seed );
    return { seed, IKHeuristicState::Success };
}

// ------------------------------------------------------------

bool Planar2RHeuristic::ValidateAndSelectElbowConfiguration(
    const VecXd& planar_seed,
    const VecXd& elbow_up,
    const VecXd& elbow_down,
    VecXd& solution ) const
{
    const auto& shoulder_limits = 
        GetChain()->GetActiveJointLimits( GetGroup().FirstIndex() );
    const auto& elbow_limits = 
        GetChain()->GetActiveJointLimits( GetGroup().Index( 1 ) );

    bool elbow_up_valid = shoulder_limits.Within( elbow_up[0] ) 
                       && elbow_limits.Within( elbow_up[1] );
    bool elbow_down_valid = shoulder_limits.Within( elbow_down[0] ) 
                         && elbow_limits.Within( elbow_down[1] );

    if ( !elbow_up_valid && !elbow_down_valid )
    {
        solution = planar_seed;
        return false;
    }
    else if ( !elbow_up_valid )
    {
        solution = elbow_down;
        return true;
    }
    else if ( !elbow_down_valid )
    {
        solution = elbow_up;
        return true;
    }

    double elbow_up_distance   = ( elbow_up - planar_seed ).cwiseAbs().sum();
    double elbow_down_distance = ( elbow_down - planar_seed ).cwiseAbs().sum();
    solution = elbow_up_distance < elbow_down_distance ? elbow_up : elbow_down;
    return true;
}

// ------------------------------------------------------------

}