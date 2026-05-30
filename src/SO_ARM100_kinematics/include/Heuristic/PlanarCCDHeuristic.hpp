#pragma once

#include "Global.hpp"
#include "IIKHeuristic.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/KinematicModel.hpp"

namespace SOArm100::Kinematics::Heuristic
{
class PlanarCCDHeuristic :
	public Model::IKJointGroupModelBase,
	public Heuristic::IIKHeuristic
{
public:
struct SolverParameters
{
	int max_iterations;
	double error_tolerance;

	SolverParameters( 
		int max_iterations = 50, 
		double error_tolerance = 5e-3 ) :
		max_iterations( max_iterations ),
		error_tolerance( error_tolerance )
	{
	}
};

explicit PlanarCCDHeuristic(
	Model::KinematicModelConstPtr model,
	Model::JointGroup planar_group,
	SolverParameters parameters = SolverParameters() );

~PlanarCCDHeuristic() = default;

PlanarCCDHeuristic( const PlanarCCDHeuristic& ) = delete;
PlanarCCDHeuristic& operator = ( const PlanarCCDHeuristic& ) = delete;

[[nodiscard]] virtual IKPresolution Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const override;

private:
SolverParameters parameters_;

std::vector< Model::JointState > InitializeJointStates() const;
bool IsUnreachable( const Vec3d& p_local_target ) const;
};
}