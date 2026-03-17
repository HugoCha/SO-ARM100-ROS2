#pragma once

#include "HybridSolver/WristCenterSolver.hpp"
#include "IKinematicsSolver.hpp"
#include "NumericJointsSolver.hpp"
#include "SolverResult.hpp"
#include "WristSolver.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
class JointChain;
class WristCenterJointsModel;
class WristModel;

class NumericWristSolver : public IKinematicsSolver
{
public:
NumericWristSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	const WristCenterJointsModel& wrist_center_model,
	const WristModel& wrist_model );

virtual SolverResult IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double discretization ) const override;

private:
struct SolverBuffer
{
	Vec3d wrist_center{};
	Mat4d wrist_center_target{};
	Mat4d wrist_target{};

	Mat4d T_num{};
	Mat4d fk_result{};

	SolverResult numeric_result;
	SolverResult wrist_result;

	SolverBuffer( int numeric, int wrist ) :
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

std::unique_ptr< NumericJointsSolver > numeric_solver_;
std::unique_ptr< WristCenterJointsSolver > wrist_center_solver_;
std::unique_ptr< WristSolver > wrist_solver_;
};
}