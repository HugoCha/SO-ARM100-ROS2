#pragma once

#include "Global.hpp"
#include "Model/KinematicModel.hpp"
#include "Scorer/IKSolutionScorer.hpp"

namespace SOArm100::Kinematics::Scorer
{
class ManipulabilityScorer : public IKSolutionScorer
{
public:
ManipulabilityScorer( Model::KinematicModelConstPtr model );

virtual double Score(
	const Solver::IKProblem& problem,
	const VecXd& solution,
	double error ) const override;

private:
Model::KinematicModelConstPtr model_;
double max_manip_;

static double ComputeMaxManipulability( Model::KinematicModelConstPtr model );
static double ComputeManipulability( Model::KinematicModelConstPtr model, const VecXd& joints );
};
}