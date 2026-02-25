#include "DLSSolver/SugiharaAdaptativeDamping.hpp"

#include "DLSSolver/AdaptativeDamping.hpp"
#include "Global.hpp"
#include "SolverType.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h>
#include <memory>

namespace SOArm100::Kinematics 
{

// ------------------------------------------------------------
// Sugihara Adaptative Damping Factory
// ------------------------------------------------------------

std::unique_ptr< AdaptativeDamping > SugiharaAdaptativeDampingFactory::Get( 
    SolverType type,
    const MatXd& jacobian,
    const VecXd& error,
    double min_damping,
    double max_damping,
    double min_sv_tolerance )
{
    switch ( type ) 
    {
        case SolverType::Position:
            return std::make_unique< PositionSugiharaAdaptativeDamping >(
                jacobian,
                min_damping,
                max_damping,
                min_sv_tolerance
            );
        case SolverType::Orientation:
            return std::make_unique< OrientationSugiharaAdaptativeDamping >(
                jacobian,
                error,
                min_damping,
                max_damping,
                min_sv_tolerance
            );
        default:
        case SolverType::Full:
            return std::make_unique< SugiharaAdaptativeDamping >(
                jacobian,
                min_damping,
                max_damping,
                min_sv_tolerance
            );
    }
}

// ------------------------------------------------------------
// Sugihara Adaptative Damping
// ------------------------------------------------------------

SugiharaAdaptativeDamping::SugiharaAdaptativeDamping( 
    const MatXd& jacobian, 
    double min_damping, 
    double max_damping,
    double min_sv_tolerance ) :
    min_damping_( min_damping ),
    max_damping_( max_damping ),
    min_sv_tolerance_( min_sv_tolerance ),
    manipulability_( 6 )
{
    Manipulability( jacobian, manipulability_ );
    Eigen::SelfAdjointEigenSolver<MatXd> eig( manipulability_ );
    // sqrt because min_sv_tolerance is for Jacobian
    // manipulability = jacobian * jacobian.transpose()
    // min_sv( manipulability ) = min_sv( jacobian )^2
    min_sv_ = sqrt( eig.eigenvalues().minCoeff() );
}

// ------------------------------------------------------------

double SugiharaAdaptativeDamping::Damping() const
{
    return min_damping_ + 
        SugiharaDamping( 
            min_damping_, 
            max_damping_, 
            min_sv_, 
            min_sv_tolerance_ );
}

// ------------------------------------------------------------

double SugiharaAdaptativeDamping::SugiharaDamping( 
    double min_damping, 
    double max_damping, 
    double min_sv, 
    double min_sv_tolerance ) const
{
    if ( min_sv >= min_sv_tolerance )
        return min_damping;
    
    double ratio = min_sv / min_sv_tolerance;
    return max_damping * ( 1.0 - ratio * ratio );
}

// ------------------------------------------------------------
// Position Sugihara Adaptative Damping
// ------------------------------------------------------------

PositionSugiharaAdaptativeDamping::PositionSugiharaAdaptativeDamping( 
    const MatXd& jacobian, 
    double min_damping, 
    double max_damping,
    double min_sv_tolerance ) :
    SugiharaAdaptativeDamping( jacobian, min_damping, max_damping, min_sv_tolerance )
{
    assert( jacobian.cols() == 3 );
}

// ------------------------------------------------------------
// Orientation Sugihara Adaptative Damping
// ------------------------------------------------------------

OrientationSugiharaAdaptativeDamping::OrientationSugiharaAdaptativeDamping(
	const MatXd& jacobian,
    const VecXd& error,
	double min_damping,
	double max_damping,
	double min_sv_tolerance ) :
    SugiharaAdaptativeDamping( jacobian, min_damping, max_damping, min_sv_tolerance ),
    error_( error )
{
    assert( jacobian.cols() == 3 );
    assert( error.size() >= 3 );
}

// ------------------------------------------------------------

double OrientationSugiharaAdaptativeDamping::Damping() const 
{
    double angle_error = error_.head<3>().norm();

    double linearization_trust = cos(angle_error / 2.0 );
    double lambda_angle_boost = max_damping_ * ( 1.0f - linearization_trust );
    
    return min_damping_ + 
        SugiharaDamping( 
            min_damping_, 
            max_damping_,
            min_sv_, 
            min_sv_tolerance_ ) + 
        lambda_angle_boost;
}

// ------------------------------------------------------------

}