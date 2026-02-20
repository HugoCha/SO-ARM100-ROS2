#pragma once

#include "BaseJointSolver.hpp"
#include "DLSSolver/DLSKinematicsSolver.hpp"
#include "Global.hpp"
#include "IKinematicsSolver.hpp"
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
	const NumericJointsModel& numeric_model,
	const WristModel& wrist_model );

virtual SolverResult IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double discretization ) const override;

private:
struct SolverBuffer
{
	Mat4d wrist_center{};
	Mat4d wrist_target{};

	Mat4d T_num{};
	Mat4d fk_result{};

	SolverResult base_result;
	SolverResult numeric_result;
	SolverResult wrist_result;

	std::vector< double > seed_joints;

	SolverBuffer( int base, int numeric, int wrist ) :
		base_result( base ),
		numeric_result( numeric ),
		wrist_result( wrist )
	{
	}

	int Size() const
	{
		return numeric_result.joints.size() + wrist_result.joints.size();
	}
};

mutable SolverBuffer buffer_;

std::shared_ptr< const JointChain > joint_chain_;
std::shared_ptr< const Mat4d > home_configuration_;

std::unique_ptr< BaseJointSolver > base_joint_presolver_;
std::unique_ptr< NumericJointsSolver > numeric_presolver_;
std::unique_ptr< WristSolver > wrist_presolver_;

std::unique_ptr< DLSKinematicsSolver > full_solver_;
};
}