#include "UniversalSolver/UniversalModel.hpp"

#include "Global.hpp"

#include "Model/Joint/Limits.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/MathUtils.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

UniversalModel::UniversalModel( 
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2 )
{
    joints_ = { joint1, joint2 };
}

// ------------------------------------------------------------

std::optional< UniversalModel > UniversalModel::ComputeModel(
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2 )
{
    std::vector< Model::JointConstPtr > universal_joints = 
    {
        joint1,
        joint2
    };

    if ( !AxesIndependent( universal_joints ) || !ComputeIntersection( universal_joints ).has_value() )
        return std::nullopt;

    return UniversalModel( 
        universal_joints[0], 
        universal_joints[1] );
}

// ------------------------------------------------------------

Mat3d UniversalModel::Recompose( const Vec2d& q ) const
{
    return ( AngleAxis( q[0], GetJoint( 0 )->Axis() )
           * AngleAxis( q[1], GetJoint( 1 )->Axis() ) ).toRotationMatrix();
}

// ------------------------------------------------------------

std::vector< Vec2d > UniversalModel::Decompose( const Mat3d& R_target ) const
{
    const Vec3d& a1 = joints_[0]->Axis();
    const Vec3d& a2 = joints_[1]->Axis();
    return {};
}

// ------------------------------------------------------------

std::vector< Vec2d > UniversalModel::Decompose( const Vec3d& b0, const Vec3d& b1 ) const
{
    auto joint0 = joints_[0];
	auto joint1 = joints_[1];

    const auto& l0 = joint0->GetLimits();
    const auto& l1 = joint1->GetLimits();

	const Vec3d& a0 = joint0->Axis();
	const Vec3d& a1 = joint1->Axis();

	double A = a1.dot( b1 ) - ( a0.dot( b1 ) ) * ( a1.dot( a0 ) );
	double B = -a1.dot( a0.cross( b1 ) );
	double C = ( a0.dot( b1 ) * a1.dot( a0 ) ) - a1.dot( b0 );

	double R = std::sqrt( A * A + B * B );
	double phi = std::atan2( B, A );
	double beta = std::acos( std::clamp( -C / ( R + 1e-9 ), -1.0, 1.0 ) );

    return { 
        ComputeSolution( phi - beta, l0, l1, a0, a1, b0, b1 ),
        ComputeSolution( phi + beta, l0, l1, a0, a1, b0, b1 ),
    };
}

// ------------------------------------------------------------

Vec2d UniversalModel::ComputeSolution(
    double theta0_sol,
    const Limits& l0,
    const Limits& l1,
    const Vec3d& a0,
    const Vec3d& a1,
    const Vec3d& b0,
    const Vec3d& b1 ) const
{
    Vec2d angles;

	angles[0] = l0.Clamp( std::remainder( theta0_sol, 2 * M_PI ) );
	auto rotation0 = AngleAxis( angles[0], a0 );

	auto rotation0_sol = AngleAxis( theta0_sol, a0 );
	Vec3d a1_prime = rotation0_sol * a1;
	Vec3d b0_prime = rotation0_sol * b0;
	
    angles[1] = l1.Clamp( AngleAroundAxis( b0_prime, b1, a1_prime ) );

    return angles;
}

// ------------------------------------------------------------

}