#include "DLS/AdaptativeDamping.hpp"

#include "Global.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------
// Linear Adaptative Damping
// ------------------------------------------------------------

double AdaptativeDamping::LinearDamping(
	double damping,
	double min_damping,
	double max_damping,
	double alpha )
{
	return std::clamp( alpha * damping, min_damping, max_damping );
}

// ------------------------------------------------------------
// Manipulability min singular value Adaptative Damping
// ------------------------------------------------------------

double AdaptativeDamping::ManipulabilityMinSV(
	const MatXd& jacobian,
	double damping,
	double min_damping,
	double max_damping,
	double min_sv_tolerance )
{
	MatXd manipulability( jacobian.rows(), jacobian.rows() );
	Manipulability( jacobian, manipulability );
	Eigen::SelfAdjointEigenSolver< MatXd > eig( manipulability );
	// sqrt because min_sv_tolerance is for Jacobian
	// manipulability = jacobian * jacobian.transpose()
	// min_sv( manipulability ) = min_sv( jacobian )^2
	double min_sv = sqrt( eig.eigenvalues().minCoeff() );

	if ( min_sv >= min_sv_tolerance )
		return std::max( 0.8 * damping, min_damping );

	if ( std::isnan( min_sv ) )
		return max_damping;

	double ratio = min_sv / min_sv_tolerance;
	return std::max( min_damping, max_damping * ( 1.0 - ratio * ratio ) );
}

// ------------------------------------------------------------
// Manipulability determinant Adaptative Damping
// ------------------------------------------------------------

double AdaptativeDamping::ManipulabilityDeterminant(
	const MatXd& jacobian,
	double min_damping,
	double max_damping,
	double epsilon )
{
	MatXd manipulability( jacobian.rows(), jacobian.rows() );
	Manipulability( jacobian, manipulability );
	double det = manipulability.determinant();
	if ( det < epsilon )
	{
		return std::max( min_damping, max_damping * ( 1 - det / epsilon ) );
	}
	return min_damping;
}

// ------------------------------------------------------------
// Manipulability trace Adaptative Damping
// ------------------------------------------------------------

double AdaptativeDamping::ManipulabilityTrace(
	const MatXd& jacobian,
	double min_damping,
	double max_damping,
	double epsilon )
{
	MatXd manipulability( jacobian.rows(), jacobian.rows() );
	Manipulability( jacobian, manipulability );
	double trace = manipulability.trace();
	if ( trace < epsilon )
	{
		return std::max( min_damping, max_damping * ( 1 - trace / epsilon ) );
	}
	return min_damping;
}

// ------------------------------------------------------------

}