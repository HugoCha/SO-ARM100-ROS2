#include "HybridKinematicsSolver.hpp"

#include "BaseJointAnalyzer.hpp"
#include "BaseJointSolver.hpp"
#include "KinematicsSolver.hpp"
#include "KinematicsUtils.hpp"
#include "NumericJointsAnalyzer.hpp"
#include "NumericJointsSolver.hpp"
#include "Twist.hpp"
#include "WristAnalyzer.hpp"
#include "WristSolver.hpp"

#include <Eigen/src/Geometry/AngleAxis.h>
#include <cmath>
#include <memory>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

HybridKinematicsSolver::HybridKinematicsSolver() :
	base_joint_solver_( nullptr ),
	numeric_solver_( nullptr ),
	wrist_solver_( nullptr )
{
}

// ------------------------------------------------------------

HybridKinematicsSolver::~HybridKinematicsSolver()
{
}

// ------------------------------------------------------------

void HybridKinematicsSolver::Initialize(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization )
{
	KinematicsSolver::Initialize( robot_model, group_name, base_frame, tip_frames, search_discretization );
	
	const auto& configuration = AnalyzeConfiguration( joint_models_, twists_, *p_home_configuration_ );

	InitializeBaseJointKinematicSolver(
		configuration.base_joint_model, 
		search_discretization );

	InitializeNumericKinematicsSolver( 
		configuration.numeric_joints_model, 
		search_discretization );

	InitializeWristKinematicsSolver(
		configuration.wrist_model, 
		base_frame,
		search_discretization );
	
	solver_flags_ = configuration.solver_flags;
}

// ------------------------------------------------------------

const HybridKinematicsSolver::SolverConfiguration HybridKinematicsSolver::AnalyzeConfiguration( 
	const std::span< const moveit::core::JointModel* const > joint_models,
	const std::span< TwistConstPtr >& twists, 
	const Mat4d& home_configuration ) const
{
	SolverConfiguration configuration;

	if ( ( configuration.wrist_model = WristAnalyzer::Analyze( 
		joint_models,
		twists, 
		home_configuration ) ) )
	{
		configuration.solver_flags |= HybridSolverFlags::Wrist;
	}

	if ( ( configuration.base_joint_model = BaseJointAnalyzer::Analyze( 
		twists[0], 
		configuration.wrist_model ) ) )
	{
		configuration.solver_flags |= HybridSolverFlags::Base;
	}

	if ( ( configuration.numeric_joints_model = NumericJointsAnalyzer::Analyze( 
		joint_models,
		twists, 
		home_configuration, 
		configuration.base_joint_model, 
		configuration.wrist_model ) ) )
	{
		configuration.solver_flags |= HybridSolverFlags::Numeric;
	}
	
	return configuration;
}

// ------------------------------------------------------------

void HybridKinematicsSolver::InitializeBaseJointKinematicSolver(	
	const std::optional< BaseJointModel >& base_joint_model,
	double search_discretization )
{
	if ( !base_joint_model )
	{
		base_joint_solver_ = nullptr;
		return;
	}
	if ( !base_joint_solver_ )
	{
		base_joint_solver_ = std::make_unique< BaseJointSolver >();
	}

	base_joint_solver_->Initialize( *base_joint_model );
}

// ------------------------------------------------------------

void HybridKinematicsSolver::InitializeNumericKinematicsSolver(	
	const std::optional< NumericJointsModel >& model,
	double search_discretization )
{
	if ( !model )
	{
		numeric_solver_ = nullptr;
		return;
	}
	if ( !numeric_solver_ )
	{
		numeric_solver_ = std::make_unique< NumericJointsSolver >();
	}

	numeric_solver_->Initialize( *model );
}

// ------------------------------------------------------------

void HybridKinematicsSolver::InitializeWristKinematicsSolver(	
	const std::optional< WristModel >& wrist_model,
	const std::string& base_frame,
	double search_discretization )
{
	if ( !wrist_model )
	{
		wrist_solver_ = nullptr;
		return;
	}
	if ( !wrist_solver_ )
	{
		wrist_solver_ = std::make_unique< WristSolver >();
	}

	wrist_solver_->Initialize( *wrist_model, search_discretization );
}

// ------------------------------------------------------------

bool HybridKinematicsSolver::InverseKinematic(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	VecXd& joints ) const
{
	wrist_solver_->ComputeWristCenter( target_pose, buffer_.wrist_center );
	
	if ( ( buffer_.base_result = base_joint_solver_->IK( 
		buffer_.wrist_center, 
		seed_joints ) ).Fail() )
		return false;
	
	base_joint_solver_->FK( buffer_.base_result.base_joint, buffer_.T_base );
	buffer_.num_target = Inverse( buffer_.T_base ) * buffer_.wrist_center;
	
	if ( !( buffer_.num_result = numeric_solver_->IK( 
		buffer_.num_target , 
		seed_joints ) ).Success() )
		return false;

	numeric_solver_->FK( buffer_.num_result.joint_angles, buffer_.T_num );
	auto T_wrist_target = Inverse( buffer_.T_num ) * buffer_.num_target;
	if ( !( buffer_.wrist_result = wrist_solver_->IK( 
		T_wrist_target, 
		seed_joints ) ).Success() )
		return false;
	
	joints << buffer_.base_result.base_joint, 
			  buffer_.num_result.joint_angles, 
			  buffer_.wrist_result.joints;
	
	return true;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics
