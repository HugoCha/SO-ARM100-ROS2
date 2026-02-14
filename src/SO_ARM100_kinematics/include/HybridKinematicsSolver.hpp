#pragma once

#include "Global.hpp"

#include "BaseJointModel.hpp"
#include "BaseJointSolver.hpp"
#include "KinematicsSolver.hpp"
#include "NumericJointsModel.hpp"
#include "NumericJointsSolver.hpp"
#include "NumericSolverResult.hpp"
#include "WristSolver.hpp"
#include "WristModel.hpp"

#include <cstdint>

namespace SOArm100::Kinematics
{
enum class HybridSolverFlags : u_int8_t
{
	None    = 0,
	Base    = 1 << 0,
	Wrist   = 1 << 1,
	Numeric = 1 << 2,
};

constexpr HybridSolverFlags operator | ( HybridSolverFlags a, HybridSolverFlags b ){
	return static_cast< HybridSolverFlags >(
		static_cast< uint8_t >( a ) | static_cast< uint8_t >( b )
		);
}

inline HybridSolverFlags& operator |= ( HybridSolverFlags& a, HybridSolverFlags b ){
	a = a | b;
	return a;
}

inline bool IsFlagSet( HybridSolverFlags flags, HybridSolverFlags flagToCheck ){
	return ( static_cast< uint8_t >( flags ) & static_cast< uint8_t >( flagToCheck ) ) != 0;
}

class HybridKinematicsSolver : public KinematicsSolver
{
public:
HybridKinematicsSolver();
~HybridKinematicsSolver();

virtual void Initialize(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization ) override;

protected:
virtual bool InverseKinematic(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	VecXd& joints ) const override;

private:
struct SolverBuffer
{
	Mat4d wrist_center{};
	Mat4d num_target{};
	Mat4d wrist_target{};

	Mat4d T_base{};
	Mat4d T_num{};

	BaseJointSolverResult base_result{};
	NumericSolverResult num_result{};
	WristSolverResult wrist_result{ 0 };
};

struct SolverConfiguration
{
	std::optional< BaseJointModel > base_joint_model{ std::nullopt };
	std::optional< NumericJointsModel > numeric_joints_model{ std::nullopt };
	std::optional< WristModel > wrist_model{ std::nullopt };
	HybridSolverFlags solver_flags{ HybridSolverFlags::None };
};

mutable SolverBuffer buffer_;
HybridSolverFlags solver_flags_;

std::unique_ptr< BaseJointSolver > base_joint_solver_;
std::unique_ptr< NumericJointsSolver > numeric_solver_;
std::unique_ptr< WristSolver > wrist_solver_;

[[nodiscard]] const SolverConfiguration AnalyzeConfiguration(
	const JointChain& joint_chain,
	const Mat4d& home_configuration ) const;

void InitializeBaseJointKinematicSolver(
	const std::optional< BaseJointModel >& base_joint_model,
	double search_discretization );

void InitializeNumericKinematicsSolver(
	const std::optional< NumericJointsModel >& model,
	double search_discretization );

void InitializeWristKinematicsSolver(
	const std::optional< WristModel >& wrist_model,
	const std::string& base_frame,
	double search_discretization );
};
}
