#pragma once

#include "Global.hpp"

#include "DLSSolver/DLSKinematicsSolver.hpp"
#include "IKinematicsHeuristic.hpp"
#include "IKinematicsSolver.hpp"
#include "Joint/JointChain.hpp"
#include "WristModel.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
struct SolverResult;

class WristSolver : 
	public IKinematicsSolver, 
	public IKinematicsHeuristic
{
public:
WristSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	const WristModel& wrist_model );

virtual SolverResult Heuristic(
	const Mat4d& target,
	const std::span< const double >& seed_joints,
	double search_discretization ) const override;

virtual SolverResult IK(
	const Mat4d& target,
	const std::span< const double >& seed_joints,
	double search_discretization ) const override;

void ComputeWristCenter( const Mat4d& target, Mat4d& wrist_center ) const;

const WristModel* GetWristModel() const {
	return wrist_model_.get();
}

private:
std::shared_ptr< const JointChain > joint_chain_;
std::shared_ptr< const Mat4d > home_configuration_;

WristModelUniqueConstPtr wrist_model_;
std::unique_ptr< DLSKinematicsSolver > dls_wrist_solver_;

SolverResult SolveAnalytical( 
	const JointChain& joint_chain, 
	const Mat3d& R_target,
	const std::span< const double >& seed_joints ) const;
SolverResult SolveRevolute1( 
	const JointChain& joint_chain, 
	const Mat3d& R_target,
	const std::span< const double >& seed_joints ) const;
SolverResult SolveRevolute2( 
	const JointChain& joint_chain, 
	const Mat3d& R_target,
	const std::span< const double >& seed_joints ) const;
SolverResult SolveRevolute3( 
	const JointChain& joint_chain, 
	const Mat3d& R_target,
	const std::span< const double >& seed_joints ) const;
SolverResult SolveNumeric( 
	const Mat4d& target, 
	const std::span< const double > seed_joints ) const;
};
}