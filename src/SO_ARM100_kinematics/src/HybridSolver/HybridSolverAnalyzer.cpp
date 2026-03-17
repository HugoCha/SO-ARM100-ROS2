#include "HybridSolver/HybridSolverAnalyzer.hpp"

#include "HybridSolver/BaseJointAnalyzer.hpp"
#include "HybridSolver/HybridSolverConfiguration.hpp"
#include "HybridSolver/WristCenterJointsAnalyzer.hpp"
#include "HybridSolver/WristAnalyzer.hpp"

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

const HybridSolverConfiguration HybridSolverAnalyzer::AnalyzeConfiguration(
	const JointChain& joint_chain,
	const Mat4d& home_configuration )
{
	HybridSolverConfiguration configuration;
	if ( ( configuration.wrist_model = WristAnalyzer::Analyze(
			   joint_chain,
			   home_configuration ) ).has_value() )
	{
		configuration.solver_flags |= HybridSolverFlags::Wrist;
	}

	if ( ( configuration.base_joint_model = BaseJointAnalyzer::Analyze(
			   joint_chain,
			   configuration.wrist_model ) ).has_value() )
	{
		configuration.solver_flags |= HybridSolverFlags::Base;
	}

	if ( ( configuration.wrist_center_joints_model = WristCenterJointsAnalyzer::Analyze(
			   joint_chain,
			   home_configuration,
			   configuration.base_joint_model,
			   configuration.wrist_model ) ).has_value() )
	{
		configuration.solver_flags |= HybridSolverFlags::Numeric;
	}

	return configuration;
}

// ------------------------------------------------------------

}