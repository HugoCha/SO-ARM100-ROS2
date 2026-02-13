#pragma once

#include "NumericJointsModel.hpp"
#include "NumericSolverResult.hpp"

namespace SOArm100::Kinematics
{
class NumericJointsSolver
{
public:
void Initialize( const NumericJointsModel& numeric_joint_model );

[[nodiscard]] NumericSolverResult IK( const Mat4d& target_pose, const std::span< const double >& seed_joints );
void FK( const VecXd& joints, Mat4d& fk ) const;

private:
NumericJointsModelUniqueConstPtr p_numeric_joints_model_;
};
}