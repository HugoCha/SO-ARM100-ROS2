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

bool ValidateAndSelectCandidates(
	double alpha,
	double beta,
	std::vector< Vec1d >& valid_candidates ) const;
};
}