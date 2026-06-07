#pragma once

#include "Global.hpp"

#include "Heuristic/IIKHeuristic.hpp"
#include "Model/Geometry/Base3d.hpp"
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


static Model::Base3d ComputeBaseReference(
	Model::KinematicModelConstPtr model,
	const Model::JointGroup& revolute_base_group );

static double ComputeAlpha(
	const Vec3d& axis,
	const Vec3d& ref_direction,
	const Vec3d& r_proj );

static double ComputeBeta(
	double shoulder_offset,
	const Vec3d& r_proj );

std::vector< double > EvaluateCandidates( double seed, double alpha, double beta ) const;
};
}