#pragma once

#include "KinematicsSolver.hpp"
#include "Types.hpp"

namespace SOArm100::Kinematics
{
class HybridKinematicsSolver : public KinematicsSolver
{
public:
HybridKinematicsSolver();
~HybridKinematicsSolver();

virtual bool InverseKinematic(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	VecXd& joints ) const override;
};
}
