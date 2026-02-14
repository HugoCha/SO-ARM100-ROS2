#pragma once

#include "Global.hpp"

#include "BaseJointModel.hpp"
#include "Joint.hpp"

#include <span>

namespace SOArm100::Kinematics
{
class JointChain;

enum class BaseJointSolverState
{
	None,
	Success,
	Unreachable,
	Singularity
};

struct BaseJointSolverResult
{
	BaseJointSolverState state{ BaseJointSolverState::None };
	VecXd base_joint{ 1 };

	inline bool Fail() const {
		return state == BaseJointSolverState::Unreachable;
	}

	inline bool Success() const {
		return state == BaseJointSolverState::Success;
	}
};

class BaseJointSolver
{
public:
void Initialize(
	const JointChain& joint_chain,
	const BaseJointModel& base_joint_model );

[[nodiscard]] BaseJointSolverResult IK(
	const Mat4d& wrist_center,
	const std::span< const double >& seed_joint ) const;

void FK( const VecXd& base_joint, Mat4d& fk ) const;

private:
const Joint* base_joint;
BaseJointModelUniqueConstPtr base_joint_model_;
};
}