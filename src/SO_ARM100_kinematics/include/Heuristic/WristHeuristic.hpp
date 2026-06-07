#pragma once

#include "Global.hpp"

#include "Heuristic/IIKHeuristic.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include "SphericalSolver/SphericalSolver.hpp"
#include "UniversalSolver/UniversalSolver.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{
enum class WristTopology;
}

namespace SOArm100::Kinematics::Heuristic
{
class WristHeuristic :
	public Model::IKJointGroupModelBase,
	public IIKHeuristic
{
public:
WristHeuristic(
	Model::KinematicModelConstPtr model,
	Model::JointGroup wrist_group );

virtual IKPresolution Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const override;

private:
std::unique_ptr< Solver::UniversalSolver > universal_solver_;
std::unique_ptr< Solver::SphericalSolver > spherical_solver_;

Mat4d ComputeWristCenter(
	const VecXd& seed,
	const Mat4d& target ) const;

Model::WristTopology GetWristTopology() const;

IKPresolution SolveRevolute1( const VecXd& seed, const Mat3d& R_wrist_target ) const;
IKPresolution SolveRevolute2( const VecXd& seed, const Mat3d& R_wrist_target ) const;
IKPresolution SolveRevolute3( const VecXd& seed, const Mat3d& R_wrist_target ) const;
};
}