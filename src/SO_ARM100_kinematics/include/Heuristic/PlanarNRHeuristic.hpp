#pragma once

#include "Heuristic/IIKHeuristic.hpp"
#include "Model/IKJointGroupModelBase.hpp"
#include <memory>

namespace SOArm100::Kinematics::Heuristic
{
class PlanarNRHeuristic : public Model::IKJointGroupModelBase, IIKHeuristic
{
public:
PlanarNRHeuristic(
	Model::KinematicModelConstPtr model,
	Model::JointGroup planar_group );

virtual IKPresolution Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const override;

private:
std::unique_ptr< IIKHeuristic > planar_heuristic_;

static std::unique_ptr< IIKHeuristic > InitializePlanarHeuristic(
    Model::KinematicModelConstPtr model,
	Model::JointGroup planar_group );
};
}