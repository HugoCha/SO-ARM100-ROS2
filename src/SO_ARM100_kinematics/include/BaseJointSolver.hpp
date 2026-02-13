#pragma once

#include "Global.hpp"

#include "BaseJointModel.hpp"

#include <memory>
#include <span>

namespace SOArm100::Kinematics 
{
enum class BaseJointSolverState
{
    None,
    Success,
    Unreachable,
    Singularity
};

struct BaseJointSolverResult
{
BaseJointSolverState state{BaseJointSolverState::None};
VecXd base_joint{1};

inline bool Fail() const {
    return state == BaseJointSolverState::Unreachable;
}

inline bool Success() const {
    return state == BaseJointSolverState::Success;
}
};

class BaseJointSolver
{

public:
inline void Initialize( const BaseJointModel& base_joint_model ) {
    p_base_joint_model_ = std::make_unique< const BaseJointModel >( base_joint_model );
}

[[nodiscard]] BaseJointSolverResult IK( const Vec3d& wrist_center, const std::span< const double >& seed_joint ) const;
void FK( const VecXd& base_joint, Mat4d& fk ) const;

private:
BaseJointModelUniqueConstPtr p_base_joint_model_;
};
}