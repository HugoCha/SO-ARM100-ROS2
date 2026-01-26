#include "HybridKinematicsSolver.hpp"

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
  geometry_msgs::msg::Pose target_pose,
  std::vector<double> & joint_angles)
{
  return false;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics
