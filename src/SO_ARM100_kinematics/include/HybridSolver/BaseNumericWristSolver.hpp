#pragma once

#include "BaseJointSolver.hpp"
#include "Global.hpp"
#include "HybridSolver/WristCenterSolver.hpp"
#include "IKinematicsSolver.hpp"
#include "WristCenterSolver.hpp"
#include "NumericJointsSolver.hpp"
#include "SolverResult.hpp"
#include "WristSolver.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
class BaseJointModel;
class JointChain;
class NumericJointsModel;
class WristModel;

class BaseNumericWristSolver : public IKinematicsSolver
{
public:
BaseNumericWristSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	const BaseJointModel& base_model,
	const WristCenterJointsModel& wrist_center_model,
	const WristModel& wrist_model );

BaseNumericWristSolver( const BaseNumericWristSolver& ) = delete;
BaseNumericWristSolver& operator = ( const BaseNumericWristSolver& ) = delete;

BaseNumericWristSolver( BaseNumericWristSolver&& ) noexcept = default;
BaseNumericWristSolver& operator = ( BaseNumericWristSolver&& ) noexcept = default;

~BaseNumericWristSolver() = default;

virtual SolverResult IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double discretization ) const override;

private:
std::shared_ptr< const JointChain > joint_chain_;
std::shared_ptr< const Mat4d > home_configuration_;

std::unique_ptr< BaseJointSolver > base_joint_presolver_;
std::unique_ptr< WristCenterJointsSolver > wrist_center_presolver_;
std::unique_ptr< WristSolver > wrist_presolver_;

std::unique_ptr< NumericJointsSolver > full_solver_;

std::vector< double > GetHeuristicJoints( 
	const SolverResult& base_result,
	const SolverResult& wrist_center_result,
	const SolverResult& wrist_result ) const;
};
}