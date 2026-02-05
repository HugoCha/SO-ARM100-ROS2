#include "HybridKinematicsSolver.hpp"

#include "Converter.hpp"
#include "KinematicsUtils.hpp"
#include "MatrixExponential.hpp"
#include "Twist.hpp"

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

HybridKinematicsSolver::HybridKinematicsSolver()
{
}

// ------------------------------------------------------------

HybridKinematicsSolver::~HybridKinematicsSolver()
{
}

// ------------------------------------------------------------

bool HybridKinematicsSolver::InverseKinematic(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	VecXd& joints ) const
{
	return false;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics
