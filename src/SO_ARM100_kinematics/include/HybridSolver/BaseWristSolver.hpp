#pragma once

#include "Global.hpp"

#include "BaseJointSolver.hpp"
#include "IKinematicsSolver.hpp"
#include "SolverResult.hpp"
#include "WristSolver.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
class BaseJointModel;
class JointChain;
class WristModel;

class BaseWristSolver : public IKinematicsSolver
{
public:
BaseWristSolver(     
    JointChain joint_chain, 
    const BaseJointModel& base_model, 
    const WristModel& wrist_model );

virtual SolverResult IK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
    double discretization ) const override;

private:
struct SolverBuffer
{
	Mat4d wrist_center{};
	Mat4d wrist_target{};

	Mat4d T_base{};

	SolverResult base_result;
	SolverResult wrist_result;

    SolverBuffer( int base, int wrist ) :
        base_result( base ),
        wrist_result( wrist )
    {}

    int Size() const
    {
        return base_result.joints.size() + wrist_result.joints.size();
    }
};

mutable SolverBuffer buffer_;

std::unique_ptr< BaseJointSolver > base_joint_solver_;
std::unique_ptr< WristSolver > wrist_solver_;
};
}