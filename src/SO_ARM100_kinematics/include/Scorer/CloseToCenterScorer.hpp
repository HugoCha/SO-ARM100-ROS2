#pragma once

#include "Model/KinematicModel.hpp"
#include "Scorer/IKSolutionScorer.hpp"

namespace SOArm100::Kinematics::Scorer
{
class CloseToCenterScorer : public IKSolutionScorer
{
public:
CloseToCenterScorer( Model::KinematicModelConstPtr model );

virtual double Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const override;

private:
Model::KinematicModelConstPtr model_;
double max_distance_;

static double ComputeMaxDistance( Model::KinematicModelConstPtr model );
};
}