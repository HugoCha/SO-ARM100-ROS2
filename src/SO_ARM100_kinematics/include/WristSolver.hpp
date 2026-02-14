#pragma once

#include "Global.hpp"

#include "DLSKinematicsSolver.hpp"
#include "JointChain.hpp"
#include "WristModel.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
enum class WristSolverState
{
	None,
	Success,
	Singularity,
	Unreachable,
};

struct WristSolverResult
{
	WristSolverState state{ WristSolverState::None };
	VecXd joints{};

	WristSolverResult( int size ) :
		joints( size )
	{
	}

	[[nodiscard]] inline bool Success() const {
		return state == WristSolverState::Success;
	}

	[[nodiscard]] inline bool Unreachable() const {
		return state == WristSolverState::Success;
	}
};

class WristSolver
{
public:
[[nodiscard]] WristSolverResult IK( const Mat4d& target_in_wrist, const std::span< const double > seed_joints ) const;

void ComputeWristCenter( const Mat4d& target, Mat4d& wrist_center ) const;

void Initialize(
	const JointChain& joint_chain,
	const WristModel& wrist_model,
	double search_discretization );

private:
std::unique_ptr< const JointChain > joint_chain_;
WristModelUniqueConstPtr wrist_model_;
std::unique_ptr< DLSKinematicsSolver > dls_wrist_solver_;

WristSolverResult SolveRevolute1( const JointChain& joint_chain, const Mat3d& R_target_in_wrist ) const;
WristSolverResult SolveRevolute2( const JointChain& joint_chain, const Mat3d& R_target_in_wrist ) const;
WristSolverResult SolveRevolute3( const JointChain& joint_chain, const Mat3d& R_target_in_wrist ) const;
WristSolverResult SolveNumeric( const Mat4d& target_in_wrist, const std::span< const double > seed_joints ) const;
};
}