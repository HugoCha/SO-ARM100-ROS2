#include "HybridSolver/NumericJointsAnalyzer.hpp"

#include "HybridSolver/BaseJointModel.hpp"
#include "HybridSolver/NumericJointsModel.hpp"
#include "HybridSolver/WristModel.hpp"
#include "Joint/JointChain.hpp"
#include "Utils/KinematicsUtils.hpp"

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

const Mat4d ComputeReducedHome(
	const Mat4d& full_home,
	const std::optional< WristModel >& wrist_model )
{
	if ( !wrist_model )
		return full_home;
	return full_home * wrist_model->tcp_in_wrist_at_home_inv;
}

// ------------------------------------------------------------

std::optional< NumericJointsModel > NumericJointsAnalyzer::Analyze(
	const JointChain& joint_chain,
	const Mat4d& home_configuration,
	const std::optional< BaseJointModel >& base_joint,
	const std::optional< WristModel >& wrist_model )
{
	int numeric_count = joint_chain.GetActiveJointCount();
	int numeric_start_index = 0;
	int wrist_count = !wrist_model ? 0 : wrist_model->active_joint_count;

	numeric_count = numeric_count - numeric_start_index - wrist_count;

	if ( numeric_count <= 0 )
		return std::nullopt;

	NumericJointsModel numeric_joint_model;
	numeric_joint_model.start_index = numeric_start_index;
	numeric_joint_model.count = numeric_count;

	if ( numeric_count == joint_chain.GetActiveJointCount() )
	{
		numeric_joint_model.home_configuration = home_configuration;
	}
	else
	{
		numeric_joint_model.home_configuration = ComputeReducedHome( home_configuration, wrist_model );
	}

	return numeric_joint_model;
}

// ------------------------------------------------------------

}