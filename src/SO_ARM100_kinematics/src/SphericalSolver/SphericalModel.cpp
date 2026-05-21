#include "SphericalSolver/SphericalModel.hpp"

#include "Global.hpp"

#include "Utils/KinematicsUtils.hpp"
#include "Utils/MathUtils.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

SphericalModel::SphericalModel( 
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2,
	const Model::JointConstPtr& joint3 ) :
    cache_( ComputeCache( joint1, joint2, joint3 ) )
{
    joints_ = { joint1, joint2, joint3 };
}

// ------------------------------------------------------------

std::optional< SphericalModel > SphericalModel::ComputeModel(
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2,
	const Model::JointConstPtr& joint3 )
{
    std::vector< Model::JointConstPtr > spherical_joints = 
    {
        joint1,
        joint2,
        joint3
    };

    if ( !AxesIndependent( spherical_joints ) || !ComputeIntersection( spherical_joints ).has_value() )
        return std::nullopt;

    return SphericalModel( 
        spherical_joints[0], 
        spherical_joints[1], 
        spherical_joints[2] );
}

// ------------------------------------------------------------

SphericalModel::Cache SphericalModel::ComputeCache( 
    Model::JointConstPtr joint1, 
    Model::JointConstPtr joint2, 
    Model::JointConstPtr joint3 )
{
    Cache c;

    const Vec3d& a1 = joint1->Axis();
    const Vec3d& a2 = joint2->Axis();
    const Vec3d& a3 = joint3->Axis();

    c.alpha = a1.dot( a2.cross( a3 ) );
    c.beta  = ( a1.dot( a2 ) ) * ( a2.dot( a3 ) ) - a1.dot( a3 );
    c.gamma = ( a1.dot( a2 ) ) * ( a2.dot( a3 ) );

    c.R_sq = c.alpha * c.alpha + c.beta * c.beta;
    c.R = std::sqrt( c.R_sq );

    c.phi = std::atan2( c.beta, c.alpha + 1e-9 );

    return c;
}

// ------------------------------------------------------------

Mat3d SphericalModel::Recompose( const Vec3d& q ) const
{
    return ( AngleAxis( q[0], GetJoint( 0 )->Axis() )
           * AngleAxis( q[1], GetJoint( 1 )->Axis() )
           * AngleAxis( q[2], GetJoint( 2 )->Axis() ) ).toRotationMatrix();
}

// ------------------------------------------------------------

std::vector< Vec3d > SphericalModel::Decompose( const Mat3d& R_target ) const
{
    const Vec3d& a1 = joints_[0]->Axis();
    const Vec3d& a2 = joints_[1]->Axis();
    const Vec3d& a3 = joints_[2]->Axis();

    double C = a1.transpose() * R_target * a3;
    double K = C - cache_.gamma;
    
    if ( K * K > cache_.R_sq + 1e-6 )
        return ComputeClosest( R_target, K );

    double K_clamped = std::clamp( K, -cache_.R, cache_.R );
    double psi = std::asin( K_clamped / ( cache_.R + 1e-9 ) );
    double theta2_sol1 = WrapAngle( cache_.phi + psi );
    double theta2_sol2 = WrapAngle( cache_.phi + M_PI - psi );

    return {
        ComputeSolution( theta2_sol1, R_target ),
        ComputeSolution( theta2_sol2, R_target )
    };
}

// ------------------------------------------------------------

std::vector< Vec3d > SphericalModel::ComputeClosest(
    const Mat3d& R_target,
    double K ) const
{
    double theta2 = std::atan2( cache_.alpha, ( K >= 0 ? -cache_.beta : cache_.beta ) + 1e-9 );
    theta2 = joints_[1]->GetLimits().Clamp( theta2 );
    
    return { ComputeSolution( theta2, R_target ) };
}

// ------------------------------------------------------------

Vec3d SphericalModel::ComputeSolution(
    double theta2,
    const Mat3d& R_target ) const
{
    const Vec3d& a1 = joints_[0]->Axis();
    const Vec3d& a2 = joints_[1]->Axis();
    const Vec3d& a3 = joints_[2]->Axis();
    
    Mat3d R_theta2 = AngleAxis( theta2, a2 ).toRotationMatrix();
    double theta1 = SolveTheta1( R_target, R_theta2, a1, a3 );
    double theta3 = SolveTheta3( R_target, R_theta2, a1, a3 );
    
    return Vec3d{ theta1, theta2, theta3 };
}

// ------------------------------------------------------------

double SphericalModel::SolveTheta1( 
    const Mat3d& R_target, 
    const Mat3d& R_theta2,
    const Vec3d& a1,
    const Vec3d& a3 )
{
    const Vec3d& v = R_theta2 * a3;
    const Vec3d& w = R_target * a3;

    return SolveSingleAxisEquation( a1, v, w );
}

// ------------------------------------------------------------

double SphericalModel::SolveTheta3( 
    const Mat3d& R_target, 
    const Mat3d& R_theta2,
    const Vec3d& a1,
    const Vec3d& a3 )
{
    const Vec3d& v = R_theta2.transpose() * a1;
    const Vec3d& w = R_target.transpose() * a1;

    return SolveSingleAxisEquation( -a3, v, w );
}

// ------------------------------------------------------------

double SphericalModel::SolveSingleAxisEquation( 
    const Vec3d& axis, 
    const Vec3d& v, 
    const Vec3d& w )
{
    const Vec3d& vperp = v - v.dot( axis ) * axis;
    const Vec3d& wperp = w - w.dot( axis ) * axis;

    return std::atan2( wperp.dot( axis.cross( vperp ) ), wperp.dot( vperp ) + 1e-9 );
}

// ------------------------------------------------------------

bool SphericalModel::IsSingular( const Mat3d& R_canonical, double tol )
{
	return SingularityMargin( R_canonical ) < tol;
}

// ------------------------------------------------------------

double SphericalModel::SingularityMargin( const Mat3d& R_canonical )
{
	const double cos_t2 = std::sqrt(
		R_canonical( 0, 0 ) * R_canonical( 0, 0 )
		+ R_canonical( 0, 1 ) * R_canonical( 0, 1 ) );
	return cos_t2;
}

// ------------------------------------------------------------

}