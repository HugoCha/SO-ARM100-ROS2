#pragma once

#include "FABRIKSolver/FabrikSolver.hpp"
#include "Global.hpp"

#include "HybridSolver/WristCenterJointsModel.hpp"
#include "IKinematicsHeuristic.hpp"
#include "IKinematicsSolver.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
class JointChain;

class WristCenterJointsSolver : 
    public IKinematicsSolver,
    public IKinematicsHeuristic 
{
public:
    WristCenterJointsSolver(
        std::shared_ptr< const JointChain > joint_chain,
        std::shared_ptr< const Mat4d > home_configuration,
        const WristCenterJointsModel& wrist_center_model );
    
    WristCenterJointsSolver( const WristCenterJointsSolver& ) = delete;
    WristCenterJointsSolver& operator = ( const WristCenterJointsSolver& ) = delete;
    
    WristCenterJointsSolver( WristCenterJointsSolver&& ) noexcept = default;
    WristCenterJointsSolver& operator = ( WristCenterJointsSolver&& ) noexcept = default;
    
    ~WristCenterJointsSolver() = default;

    const WristCenterJointsModel* GetWristCenterModel() const {
        return wrist_center_model_.get();
    }

    int GetJointStartIndex() const {
        return wrist_center_model_->start_index;
    }

    int GetJointCount() const {
        return wrist_center_model_->count;
    }

    virtual SolverResult Heuristic(
        const Mat4d& target,
        const std::span< const double >& seed_joints,
        double search_discretization ) const override;
    
    virtual SolverResult IK(
        const Mat4d& target,
        const std::span< const double >& seed_joints,
        double search_discretization ) const override;

    bool FK( const VecXd& joints, Mat4d& fk ) const;

private:
std::unique_ptr< FABRIKKinematicsSolver > fabrik_solver_;
std::unique_ptr< const WristCenterJointsModel > wrist_center_model_;
};
}