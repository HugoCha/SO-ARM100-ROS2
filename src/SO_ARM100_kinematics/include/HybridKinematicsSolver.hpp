#pragma once

#include "KinematicsSolver.hpp"

namespace SOArm100::Kinematics
{
class HybridKinematicsSolver : public KinematicsSolver
{
public:
  HybridKinematicsSolver();
  ~HybridKinematicsSolver();

  virtual bool InverseKinematic(
    geometry_msgs::msg::Pose target_pose,
    std::vector<double> & joint_angles) override;
};
}
