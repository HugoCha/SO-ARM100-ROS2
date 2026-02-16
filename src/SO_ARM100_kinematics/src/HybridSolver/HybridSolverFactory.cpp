#include "HybridSolver/HybridSolverFactory.hpp"

#include "HybridSolver/BaseJointSolver.hpp"
#include "HybridSolver/BaseWristSolver.hpp"
#include "HybridSolver/BaseNumericWristSolver.hpp"
#include "HybridSolver/HybridSolverConfiguration.hpp"
#include "HybridSolver/NumericJointsSolver.hpp"
#include "HybridSolver/WristSolver.hpp"
#include "IKinematicsSolver.hpp"
#include "Joint/JointChain.hpp"

#include <memory>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

std::unique_ptr< IKinematicsSolver > HybridSolverFactory::Get(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	const HybridSolverConfiguration& configuration )
{
	switch ( configuration.solver_flags )
	{
	case HybridSolverFlags::Base:
		return std::make_unique< BaseJointSolver >(
			joint_chain,
			home_configuration,
			*configuration.base_joint_model );
	case HybridSolverFlags::Wrist:
		return std::make_unique< WristSolver >(
			joint_chain,
			home_configuration,
			*configuration.wrist_model );
	case HybridSolverFlags::BaseWrist:
		return std::make_unique< BaseWristSolver >(
			joint_chain,
			home_configuration,
			*configuration.base_joint_model,
			*configuration.wrist_model
			);
	case HybridSolverFlags::BaseNumericWrist:
		return std::make_unique< BaseNumericWristSolver >(
			joint_chain,
			home_configuration,
			*configuration.base_joint_model,
			*configuration.numeric_joints_model,
			*configuration.wrist_model );
	default:
	case HybridSolverFlags::Numeric:
		return std::make_unique< NumericJointsSolver >(
			joint_chain,
			home_configuration,
			*configuration.numeric_joints_model );
	}
}

// ------------------------------------------------------------

}