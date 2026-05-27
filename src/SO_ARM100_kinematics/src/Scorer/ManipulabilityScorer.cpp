#include "Scorer/ManipulabilityScorer.hpp"

#include "Global.hpp"

#include "Solver/IKProblem.hpp"
#include "Solver/IKSolution.hpp"
#include "Utils/KinematicsUtils.hpp"

namespace SOArm100::Kinematics::Scorer
{

// ------------------------------------------------------------

ManipulabilityScorer::ManipulabilityScorer(  Model::KinematicModelConstPtr model ) :
    model_( model ),
    max_manip_( ComputeMaxManipulability( model ) )
{}

// ------------------------------------------------------------

double ManipulabilityScorer::Score(
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const
{
    double manip = ComputeManipulability( model_, solution.joints );

    return std::clamp( 1 - ( manip ) / ( max_manip_ ), 0.0, 1.0 );
}

// ------------------------------------------------------------

double ManipulabilityScorer::ComputeMaxManipulability( Model::KinematicModelConstPtr model )
{
    const auto& chain = model->GetChain();
    random_numbers::RandomNumberGenerator rng;
    double max_manip = 1e-9;

    for ( int i = 0; i < 1000; i++ )
    {
        VecXd random = chain->RandomValidJoints( rng, 0 );
        double manip = ComputeManipulability( model, random );
        if ( manip > max_manip )
            max_manip = manip;
    }

    return max_manip;
}

// ------------------------------------------------------------

double ManipulabilityScorer::ComputeManipulability( Model::KinematicModelConstPtr model, const VecXd& joints )
{
    const int n_joints = model->GetChain()->GetActiveJointCount();
    MatXd J, M;

    SpaceJacobian( 
        *model->GetChain(), 
        joints, 
        J );
    
    Manipulability( J, M );

    return sqrt( std::max( 1e-9, M.trace() ) );
}

// ------------------------------------------------------------

}