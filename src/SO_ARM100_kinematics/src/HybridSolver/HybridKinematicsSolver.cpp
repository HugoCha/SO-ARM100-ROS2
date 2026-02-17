#include "HybridSolver/HybridKinematicsSolver.hpp"

#include "HybridSolver/HybridSolverAnalyzer.hpp"
#include "HybridSolver/HybridSolverConfiguration.hpp"
#include "HybridSolver/HybridSolverFactory.hpp"
#include "SolverResult.hpp"

#include <Eigen/src/Geometry/AngleAxis.h>
#include <cmath>
#include <memory>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

HybridKinematicsSolver::HybridKinematicsSolver() :
	solver_( nullptr )
{
}

// ------------------------------------------------------------

HybridKinematicsSolver::~HybridKinematicsSolver()
{
}

// ------------------------------------------------------------

void HybridKinematicsSolver::Initialize(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization )
{
	KinematicsSolver::Initialize( robot_model, group_name, base_frame, tip_frames, search_discretization );
	search_discretization_ = search_discretization;

	const HybridSolverConfiguration& configuration = HybridSolverAnalyzer::AnalyzeConfiguration(
		*joint_chain_,
		*home_configuration_ );

	solver_ = std::move( HybridSolverFactory::Get(
							 joint_chain_,
							 home_configuration_,
							 configuration ) );
}

// ------------------------------------------------------------

void HybridKinematicsSolver::Initialize(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	double search_discretization ) 
{
	KinematicsSolver::Initialize( joint_chain, home_configuration, search_discretization );
	search_discretization_ = search_discretization;

	HybridSolverConfiguration configuration = HybridSolverAnalyzer::AnalyzeConfiguration(
		*joint_chain_,
		*home_configuration_ );

	solver_ = std::move( HybridSolverFactory::Get(
							 joint_chain_,
							 home_configuration_,
							 configuration ) );
}

// ------------------------------------------------------------

bool HybridKinematicsSolver::InverseKinematicImpl(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double* joints ) const
{
	assert( solver_ );

	if ( KinematicsSolver::IsUnreachable( target_pose ) )
		return false;

	auto result = solver_->IK( target_pose, seed_joints, search_discretization_ );
	std::copy( result.joints.begin(), result.joints.end(), joints );

	return result.Success() || result.Singularity();
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics
