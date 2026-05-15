#pragma once

#include "Global.hpp"
#include "Model/Joint/Joint.hpp"

#include <optional>
#include <vector>

namespace SOArm100::Kinematics::Model
{
class EulerModel
{
public:
[[nodiscard]] static std::optional< EulerModel > ComputeModel(
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2,
	const Model::JointConstPtr& joint3 );

[[nodiscard]] std::span< const Model::JointConstPtr > GetJoints() const {
	return joints_;
}

[[nodiscard]] const Model::JointConstPtr GetJoint( int i ) const {
	return joints_[i];
}

[[nodiscard]] const Mat3d& Q()  const {
	return Q_;
}

[[nodiscard]] Vec3d ToCanonical( const Vec3d& v_physical ) const {
	return Q_.transpose() * v_physical;
}

[[nodiscard]] Vec3d ToPhysical( const Vec3d& v_canonical ) const {
	return Q_ * v_canonical;
}

[[nodiscard]] Mat3d ToCanonical( const Mat3d& R_physical ) const {
	return Q_.transpose() * R_physical * Q_;
}

[[nodiscard]] Mat3d ToPhysical( const Mat3d& R_canonical ) const {
	return Q_ * R_canonical * Q_.transpose();
}

[[nodiscard]] Mat3d RecomposeCanonical( const Vec3d& q ) const;
[[nodiscard]] Mat3d RecomposePhysical( const Vec3d& q ) const;

[[nodiscard]] std::vector< Vec3d > DecomposeCanonical( const Mat3d& R_Canonical ) const;
[[nodiscard]] std::vector< Vec3d > DecomposePhysical( const Mat3d& R_Physical ) const;

/// Check if a canonical rotation is near XYZ gimbal lock (θ₂ ≈ ±π/2).
[[nodiscard]] bool IsSingular( const Mat3d& R_canonical, double tol = 0.05 ) const;

/// Measure distance from singularity (0 = at singularity, 1 = far away).
[[nodiscard]] double SingularityMargin( const Mat3d& R_canonical ) const;

private:
enum class EulerAxis
{
	X = 0,
	Y = 1,
	Z = 2
};

Mat3d Q_;
bool indirect_;
std::vector< Model::JointConstPtr > joints_;

EulerModel(
	const Mat3d& basis_matrix,
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2,
	const Model::JointConstPtr& joint3 );

static Mat3d BuildBasisChangeMatrix(
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2,
	const Model::JointConstPtr& joint3 );
};
}