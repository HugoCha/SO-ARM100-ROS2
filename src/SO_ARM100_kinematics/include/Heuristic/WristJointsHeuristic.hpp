#pragma once 

#include "Global.hpp"

#include "Heuristic/IKHeuristic.hpp"
#include "HybridSolver/WristModel.hpp"
#include "Model/JointGroup.hpp"

namespace SOArm100::Kinematics::Heuristic
{
class WristJointsHeuristic : public IKHeuristic
{
public:
WristJointsHeuristic( Model::KinematicModelConstPtr model, Model::JointGroup wrist_group );

virtual IKPresolution Presolve( 
    const Solver::IKProblem& problem,
    const Solver::IKRunContext& context ) const override;

private:
Model::JointGroup wristless_group_;
Model::JointGroup wrist_group_;

static Model::JointGroup ComputeWristlessGroup( 
    Model::KinematicModelConstPtr model, 
    const Model::JointGroup wrist_group );

Mat4d ComputeWristCenter(	
    const VecXd& seed, 
	const Mat4d& target ) const;

WristType GetWristType() const;

IKPresolution SolveRevolute1( const VecXd& seed, const Mat3d& R_wrist_target ) const;
IKPresolution SolveRevolute2( const VecXd& seed, const Mat3d& R_wrist_target ) const;
IKPresolution SolveRevolute3( const VecXd& seed, const Mat3d& R_wrist_target ) const;
};
}