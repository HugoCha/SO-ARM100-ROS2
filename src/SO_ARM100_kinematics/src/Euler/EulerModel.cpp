#include "Euler/EulerModel.hpp"

#include "Global.hpp"
#include "Model/Joint/Joint.hpp"
#include "Utils/MathUtils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <optional>


namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

EulerModel::EulerModel(
	const Mat3d& basis_matrix,
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2,
	const Model::JointConstPtr& joint3 ) :
	Q_( basis_matrix ),
	joints_( { joint1, joint2, joint3 } )
{
}

// ------------------------------------------------------------

std::optional< EulerModel > EulerModel::ComputeModel(
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2,
	const Model::JointConstPtr& joint3 )
{
	Mat3d Q = BuildBasisChangeMatrix( joint1, joint2, joint3 );

	if ( OrthogonalityError( Q ) > 1e-3 )
	{
		return std::nullopt;
	}

	return EulerModel( Q, joint1, joint2, joint3 );
}

// ------------------------------------------------------------

Mat3d EulerModel::BuildBasisChangeMatrix(
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2,
	const Model::JointConstPtr& joint3 )
{
	Mat3d Q;

	Q.col( 0 ) = joint1->Axis().normalized();
	Q.col( 1 ) = joint2->Axis().normalized();
	Q.col( 2 ) = joint3->Axis().normalized();

	return Q;
}

// ------------------------------------------------------------

Vec3d EulerModel::DecomposeCanonical( const Mat3d& R_canonical ) const
{
	Vec3d angles = R_canonical.eulerAngles(
		( long )EulerAxis::X,
		( long )EulerAxis::Y,
		( long )EulerAxis::Z );

	return angles;
}

// ------------------------------------------------------------

Vec3d EulerModel::DecomposePhysical( const Mat3d& R_physical ) const
{
	return DecomposeCanonical( ToCanonical( R_physical ) );
}

// ------------------------------------------------------------

Mat3d EulerModel::RecomposeCanonical( const Vec3d& angles ) const
{
	return ( AngleAxis( angles[0], Vec3d::UnitX() )
	         * AngleAxis( angles[1], Vec3d::UnitY() )
	         * AngleAxis( angles[2], Vec3d::UnitZ() ) ).toRotationMatrix();
}

// ------------------------------------------------------------

Mat3d EulerModel::RecomposePhysical( const Vec3d& angles ) const
{
	return ( AngleAxis( angles[0], GetJoint( 0 )->Axis() )
	         * AngleAxis( angles[1], GetJoint( 1 )->Axis() )
	         * AngleAxis( angles[2], GetJoint( 2 )->Axis() ) ).toRotationMatrix();
}

// ------------------------------------------------------------

bool EulerModel::IsSingular( const Mat3d& R_canonical, double tol ) const
{
	return SingularityMargin( R_canonical ) < tol;
}

// ------------------------------------------------------------

double EulerModel::SingularityMargin( const Mat3d& R_canonical ) const
{
	const double cos_t2 = std::sqrt(
		R_canonical( 0, 0 ) * R_canonical( 0, 0 )
		+ R_canonical( 0, 1 ) * R_canonical( 0, 1 ) );
	return cos_t2;
}

// ------------------------------------------------------------

}