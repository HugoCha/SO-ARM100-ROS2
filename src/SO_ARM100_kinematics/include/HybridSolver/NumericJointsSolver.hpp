#pragma once

#include "DLSSolver/DLSKinematicsSolver.hpp"
#include "IKinematicsSolver.hpp"
#include "NumericJointsModel.hpp"
#include "SolverType.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
struct SolverResult;

class NumericJointsSolver : public IKinematicsSolver
{
public:
NumericJointsSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration,
	const NumericJointsModel& numeric_joint_model,
	SolverType type = SolverType::Full );

NumericJointsSolver(
	std::shared_ptr< const JointChain > joint_chain,
	std::shared_ptr< const Mat4d > home_configuration );

virtual SolverResult IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	double search_discretization ) const override;

bool FK( const VecXd& joints, Mat4d& fk ) const;

const NumericJointsModel* GetNumericJointsModel() const {
	return numeric_joints_model_.get();
}

private:
std::shared_ptr< const JointChain > joint_chain_;
std::shared_ptr< const Mat4d > home_configuration_;

std::unique_ptr< DLSKinematicsSolver > dls_solver_;
NumericJointsModelUniqueConstPtr numeric_joints_model_;

static DLSKinematicsSolver::SolverParameters InitializeParameters( SolverType type );
};
}