#pragma once

#include "Global.hpp"
#include "Model/KinematicModel.hpp"
#include "Scorer/IKSolutionScorer.hpp"
#include "Solver/IKSolution.hpp"
#include "Solver/IKProblem.hpp"
#include "Utils/KinematicsUtils.hpp"

namespace SOArm100::Kinematics::Scorer
{
class PoseErrorScorer : public IKSolutionScorer
{
struct ScorerParameters
{
    double rotation_tolerance;
    double translation_tolerance;

    double rotation_weight;
    double translation_weight;

    double violation_penalty;

    ScorerParameters()
        : rotation_tolerance( SOArm100::Kinematics::rotation_tolerance )
        , translation_tolerance( SOArm100::Kinematics::translation_tolerance )
        , rotation_weight( 1.0 )
        , translation_weight( 10.0 )
        , violation_penalty( 1e6 )
    {}
};

public:
PoseErrorScorer( 
    Model::KinematicModelConstPtr model, 
    ScorerParameters parameters = ScorerParameters() ) :
    model_( model ),
    parameters_( parameters )
{}

virtual double Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const override
{
    VecXd weighted_pose_error;
    Mat4d fk;
    model_->ComputeFK( solution.joints, fk );
    WeightedPoseError( 
        problem.target, 
        fk, 
        parameters_.rotation_weight, 
        parameters_.translation_weight, 
        weighted_pose_error );

    const double rot_tol = parameters_.rotation_weight * parameters_.rotation_tolerance;
    const double trans_tol = parameters_.translation_weight * parameters_.translation_tolerance;

    if ( ( rot_tol > 0 && weighted_pose_error.head( 3 ).norm() > rot_tol ) ||
         ( trans_tol > 0 && weighted_pose_error.tail( 3 ).norm() > trans_tol ) )
        return ( parameters_.violation_penalty * weighted_pose_error ).norm();

    return weighted_pose_error.norm();
}

private:
Model::KinematicModelConstPtr model_;
ScorerParameters parameters_;
};
}