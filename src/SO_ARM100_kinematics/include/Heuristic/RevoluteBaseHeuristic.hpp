#pragma once

#include "Global.hpp"

#include "Heuristic/IIKHeuristic.hpp"
#include "Model/IKJointGroupModelBase.hpp"

namespace SOArm100::Kinematics::Heuristic
{
class RevoluteBaseHeuristic :
	public Model::IKJointGroupModelBase,
	public IIKHeuristic
{
public:
RevoluteBaseHeuristic(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& revolute_base_group );

virtual IKPresolution Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const override;

private:
Vec3d reference_direction_;
double shoulder_offset_;

const Model::Joint* GetBaseJoint() const;
const Model::Joint* GetShoulderJoint() const;

Vec3d ComputeDirection( const Mat4d& T_tip ) const;

static double ComputeAlpha(
	const Vec3d& axis,
	const Vec3d& ref_direction,
	const Vec3d& r_proj );

static double ComputeBeta(
	double shoulder_offset,
	const Vec3d& r_proj );

bool ValidateAndSelectCandidate(
	const Vec3d& p_target,
	const VecXd& seed,
	double alpha,
	double beta,
	double& fk_error,
	Vec1d& best_candidate ) const;
};
}