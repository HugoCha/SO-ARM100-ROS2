#pragma once

#include "Global.hpp"
#include "Model/Joint/Joint.hpp"

#include <optional>
#include <vector>

namespace SOArm100::Kinematics::Model
{
class SphericalModel
{
public:
[[nodiscard]] static std::optional< SphericalModel > ComputeModel(
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2,
	const Model::JointConstPtr& joint3 );

[[nodiscard]] std::span< const Model::JointConstPtr > GetJoints() const {
	return joints_;
}

[[nodiscard]] const Model::JointConstPtr GetJoint( int i ) const {
	return joints_[i];
}

[[nodiscard]] Mat3d Recompose( const Vec3d& q ) const;
[[nodiscard]] std::vector< Vec3d > Decompose( const Mat3d& R ) const;

/// Check if a canonical rotation is near XYZ gimbal lock (θ₂ ≈ ±π/2).
[[nodiscard]] static bool IsSingular( const Mat3d& R_canonical, double tol = 0.05 );

/// Measure distance from singularity (0 = at singularity, 1 = far away).
[[nodiscard]] static double SingularityMargin( const Mat3d& R_canonical );

private:
struct Cache
{
	double alpha;
	double beta;
	double gamma;

	double R;
	double R_sq;
	double phi;
};

std::vector< Model::JointConstPtr > joints_;
Cache cache_;

SphericalModel(
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2,
	const Model::JointConstPtr& joint3 );

[[nodiscard]] std::vector< Vec3d > ComputeClosest(
	const Mat3d& R_target,
	double K ) const;

[[nodiscard]] Vec3d ComputeSolution(
	double theta2,
	const Mat3d& R_target ) const;

[[nodiscard]] static double SolveTheta1(
	const Mat3d& R_target,
	const Mat3d& R_theta2,
	const Vec3d& a1,
	const Vec3d& a3 );

[[nodiscard]] static double SolveTheta3(
	const Mat3d& R_target,
	const Mat3d& R_theta2,
	const Vec3d& a1,
	const Vec3d& a3 );

[[nodiscard]] static double SolveSingleAxisEquation(
	const Vec3d& axis,
	const Vec3d& v,
	const Vec3d& w );

[[nodiscard]] static Cache ComputeCache(
	Model::JointConstPtr joint1,
	Model::JointConstPtr joint2,
	Model::JointConstPtr joint3 );
};
}