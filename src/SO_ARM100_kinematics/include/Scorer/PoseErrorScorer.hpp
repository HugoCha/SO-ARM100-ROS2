#pragma once

#include "Global.hpp"
#include "Model/KinematicModel.hpp"
#include "Scorer/IKSolutionScorer.hpp"

namespace SOArm100::Kinematics::Scorer
{
class PoseErrorScorer : public IKSolutionScorer
{
public:
struct ScorerParameters
{
	double error_tolerance { SOArm100::Kinematics::error_tolerance };
	double violation_penalty { 1e3 };
};

PoseErrorScorer(
	Model::KinematicModelConstPtr model,
	ScorerParameters parameters );

virtual double Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const override;

private:
Model::KinematicModelConstPtr model_;
ScorerParameters parameters_;
};
}