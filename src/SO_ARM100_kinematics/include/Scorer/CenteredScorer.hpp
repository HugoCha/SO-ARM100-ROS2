#pragma once

#include "Model/KinematicModel.hpp"
#include "Scorer/IKSolutionScorer.hpp"
#include "Solver/IKSolution.hpp"
#include "Utils/Distance.hpp"

namespace SOArm100::Kinematics::Scorer
{
class CenteredScorer : public IKSolutionScorer
{
public:
CenteredScorer( Model::KinematicModelConstPtr model ) : 
    model_( model )
{}

virtual double Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const override
{
    return Utils::Distance( 
        model_->GetChain()->ActiveJointCenters(), 
        solution.joints );
}

private:
Model::KinematicModelConstPtr model_;
};
}