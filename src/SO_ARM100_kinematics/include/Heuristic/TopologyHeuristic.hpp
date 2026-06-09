#pragma once

#include "Heuristic/IIKHeuristic.hpp"
#include "Model/IKModelBase.hpp"

#include <memory>

namespace SOArm100::Kinematics::Heuristic
{
class TopologyHeuristic : public Model::IKModelBase, public IIKHeuristic
{
public:
TopologyHeuristic( Model::KinematicModelConstPtr model );

virtual IKPresolution Presolve(
	const Solver::IKProblem& problem,
	const Solver::IKRunContext& context ) const override;

private:
std::unique_ptr< IIKHeuristic > base_heuristic_;
std::unique_ptr< IIKHeuristic > planar_heuristic_;
std::unique_ptr< IIKHeuristic > wrist_heuristic_;
std::unique_ptr< IIKHeuristic > fabrik_heuristic_;
};
}