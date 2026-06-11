#include "RobotArmKinematicsSolver.hpp"

#include "Global.hpp"

#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/JointChainBuilder.hpp"
#include "Model/Joint/Twist.hpp"
#include "Model/KinematicModel.hpp"
#include "Model/KinematicTopology/KinematicTopology.hpp"
#include "Model/ReachableSpace/SkeletonTotalLengthReachableSpace.hpp"
#include "ModelAnalyzer/SkeletonAnalyzer.hpp"
#include "ModelAnalyzer/TopologyAnalyzer.hpp"
#include "PipelineSolver/PipelineSolver.hpp"
#include "PipelineSolver/PipelineSolverInitializer.hpp"
#include "PipelineSolver/PipelineSolverParameters.hpp"
#include "Solver/IIKSolver.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/StringConverter.hpp"

#include <cassert>
#include <memory>
#include <moveit/robot_model/fixed_joint_model.hpp>
#include <moveit/robot_model/joint_model.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_model/prismatic_joint_model.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/logger.hpp>
#include <string>
#include <vector>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

static rclcpp::Logger get_logger()
{
	return Logger::get();
}

// ------------------------------------------------------------

RobotArmKinematicsSolver::RobotArmKinematicsSolver() :
	model_( nullptr )
{
}

// ------------------------------------------------------------

bool RobotArmKinematicsSolver::Initialize(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization )
{
	RCLCPP_INFO( get_logger(), "Initializing RobotArmKinematicsSolver for group: %s",
	             group_name.c_str() );

	if ( !AreValidInitializeParameters( robot_model, group_name, base_frame, tip_frames, search_discretization ) )
	{
		return false;
	}

	moveit::core::RobotState state( robot_model );
	state.setToDefaultValues();
	state.update();

	const auto* joint_model_group = robot_model->getJointModelGroup( group_name );
	const int n_joints = joint_model_group->getJointModels().size();

	Mat4d home_configuration = state.getGlobalLinkTransform( tip_frames[0] ).matrix();
	auto joint_chain_builder = Model::JointChainBuilder();

	for ( const auto* joint_model : joint_model_group->getJointModels() )
	{
		const auto* parent_link = joint_model->getParentLinkModel();
		const auto* child_link = joint_model->getChildLinkModel();

		auto parent_link_global_tf = state.getGlobalLinkTransform(parent_link);
		auto child_link_global_tf = state.getGlobalLinkTransform( child_link );
		auto joint_local_tf = state.getJointTransform( joint_model );
		auto joint_global_tf = parent_link_global_tf * joint_local_tf;

		Model::Limits joint_limits;
		const auto& bounds = joint_model->getVariableBounds();
		if ( !bounds.empty() )
		{
			const auto& lim = bounds[0];
			joint_limits = Model::Limits( lim.min_position_, lim.max_position_ );
		}

		Model::Twist joint_twist;
		if ( joint_model->getType() == moveit::core::JointModel::REVOLUTE )
		{
			const auto* revolute_joint_model =
				static_cast< const moveit::core::RevoluteJointModel* >( joint_model );

			const Vec3d axis_world = joint_global_tf.rotation() * revolute_joint_model->getAxis();
			const Vec3d point_on_axis_world = joint_global_tf.translation();

			joint_twist = Model::Twist( axis_world, point_on_axis_world );
		}
		else if ( joint_model->getType() == moveit::core::JointModel::PRISMATIC )
		{
			const auto* prismatic_joint_model =
				static_cast< const moveit::core::PrismaticJointModel* >( joint_model );

			const Vec3d axis_world = joint_global_tf.rotation() * prismatic_joint_model->getAxis();
			joint_twist = Model::Twist( axis_world );
		}
		else if ( joint_model->getType() == moveit::core::JointModel::FIXED )
		{
			joint_twist = Model::Twist();
			joint_limits = Model::Limits();
		}

		joint_chain_builder.AddParentLink( parent_link->getName(), parent_link_global_tf.matrix() );
		joint_chain_builder.AddJoint( joint_model->getName(), joint_global_tf.matrix(), joint_twist, joint_limits );
		if ( child_link )
		{
			joint_chain_builder.AddChildLink( child_link->getName(), child_link_global_tf.matrix(), child_link->getJointOriginTransform().matrix() );
		}
	}
	
	auto joint_chain = joint_chain_builder.Build();
	auto skeleton = Model::SkeletonAnalyzer::Analyze( joint_chain->GetJoints(), home_configuration );
	Model::KinematicTopology topology = Model::TopologyAnalyzer::Analyze( *joint_chain, home_configuration );
	auto reachable_space =
		std::make_unique< const Model::SkeletonTotalLengthReachableSpace >( *skeleton );

	model_ = std::make_shared< Model::KinematicModel >(
		std::move( joint_chain ),
		home_configuration,
		topology,
		std::move( skeleton ),
		std::move( reachable_space ) );

	InitializeSolvers( model_ );

	std::stringstream ss;
	ss << *model_;
	RCLCPP_INFO( get_logger(), "Robot initialized\n%s", ss.str().c_str() );
	return true;
}

// ------------------------------------------------------------

void RobotArmKinematicsSolver::InitializeSolvers( const Model::KinematicModelConstPtr& model )
{
	getIK_solver_ = std::move( Solver::PipelineSolverInitializer::InitializeSinglePipeline( model ) );

	Solver::PipelineSolverParameters pipeline_params;
	pipeline_params.strategy = Solver::PipelineCompletionStrategy::WaitForAcceptableResult;
	pipeline_params.min_score_threshold = 0.25;

	searchIK_solver_ = std::move( Solver::PipelineSolverInitializer::InitializeMultiplePipeline( model, pipeline_params ) );
}

// ------------------------------------------------------------

bool RobotArmKinematicsSolver::Initialize(
	Model::KinematicModelConstPtr model,
	std::unique_ptr< Solver::IIKSolver > getIK_solver,
	std::unique_ptr< Solver::IIKSolver > searchIK_solver,
	double search_discretization )
{
	model_ = model;
	getIK_solver_ = std::move( getIK_solver );
	searchIK_solver_ = std::move( searchIK_solver );
	return true;
}


// ------------------------------------------------------------

bool RobotArmKinematicsSolver::ForwardKinematic(
	const std::span< const double >& joints,
	geometry_msgs::msg::Pose& tip_pose ) const
{
	Mat4d T_end;
	if ( ForwardKinematic( ToVecXd( joints ), T_end ) )
	{
		tip_pose = ToPoseMsg( T_end );
		return true;
	}
	return false;
}

// ------------------------------------------------------------

bool RobotArmKinematicsSolver::ForwardKinematic(
	const VecXd& joints,
	Mat4d& pose ) const
{
	if ( !model_  )
	{
		RCLCPP_ERROR( get_logger(), "Joint model not initialized." );
		return false;
	}

	return model_->ComputeFK( joints, pose );
}

// ------------------------------------------------------------

bool RobotArmKinematicsSolver::ForwardKinematic(
	const std::span< const std::string >& link_names,
	const std::span< const double >& joints,
	std::vector< geometry_msgs::msg::Pose >& poses,
	geometry_msgs::msg::Pose& tip_pose ) const
{
	if ( !model_  )
	{
		RCLCPP_ERROR( get_logger(), "Joint model not initialized." );
		return false;
	}

	std::vector< Mat4d > poses_mat( link_names.size() );
	Mat4d tip_pose_mat;
	bool success = ForwardKinematic( link_names, ToVecXd( joints ), poses_mat, tip_pose_mat );

	if ( poses.size() != poses_mat.size() )
		poses.resize( poses_mat.size() );

	for ( int i = 0; i < poses.size(); i++ )
		poses[i] = ToPoseMsg( poses_mat[i] );

	tip_pose = ToPoseMsg( tip_pose_mat );

	return success;
}

// ------------------------------------------------------------

bool RobotArmKinematicsSolver::ForwardKinematic(
	const std::span< const std::string >& link_names,
	const VecXd& joints,
	std::vector< Mat4d >& poses,
	Mat4d& tip_pose ) const
{
	if ( !model_  )
	{
		RCLCPP_ERROR( get_logger(), "Joint model not initialized." );
		return false;
	}

	const auto& chain = model_->GetChain();
	const auto& home = model_->GetHomeConfiguration();

	return chain->ComputeLinkPosesFK(
		joints,
		home,
		poses,
		tip_pose );
}

// ------------------------------------------------------------

bool RobotArmKinematicsSolver::InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	const std::span< const double >& seed_joints,
	const std::span< const double >& consistency_limits,
	long timeout_ms,
	double tolerance,
	std::vector< double >& joints ) const
{
	if ( !model_ )
	{
		RCLCPP_ERROR( get_logger(), "Joint model not initialized." );
		return false;
	}

	const int n_joints = model_->GetChain()->GetActiveJointCount();
	if ( joints.size() != n_joints )
		joints.resize( n_joints );

	return InverseKinematic(
		ToTransformMatrix( target_pose ),
		ToVecXd( seed_joints ),
		ToVecXd( consistency_limits ),
		timeout_ms,
		tolerance,
		joints.data(),
		n_joints );
}

// ------------------------------------------------------------

bool RobotArmKinematicsSolver::InverseKinematic(
	const Mat4d& target,
	const VecXd& seed,
	const VecXd& consistency,
	long timeout_ms,
	double tolerance,
	VecXd& joints ) const
{
	if ( !model_ )
	{
		RCLCPP_ERROR( get_logger(), "Joint model not initialized." );
		return false;
	}

	const int n_joints = model_->GetChain()->GetActiveJointCount();
	if ( joints.size() != n_joints )
		joints.resize( n_joints );

	return InverseKinematic(
		target,
		seed,
		consistency,
		timeout_ms,
		tolerance,
		joints.data(),
		n_joints );
}

// ------------------------------------------------------------

bool RobotArmKinematicsSolver::InverseKinematic(
	const Mat4d& target,
	const VecXd& seed,
	const VecXd& consistency,
	long timeout_ms,
	double tolerance,
	double* joints,
	int n_joints ) const
{
	if ( !getIK_solver_ || !searchIK_solver_ )
	{
		RCLCPP_ERROR( get_logger(), "Solver not initialized." );
		return false;
	}

	if ( seed.size() != n_joints )
	{
		RCLCPP_ERROR( get_logger(), "Wrong seed size." );
		return false;
	}

	Solver::IKProblem problem {
		target,
		seed,
		consistency,
		tolerance,
		timeout_ms };

	Solver::IKRunContext context;
	Solver::IKSolution solution;

	solution = ( timeout_ms == 0 ) ?
	           getIK_solver_->Solve( problem, context ) :
	           searchIK_solver_->Solve( problem, context );

	joints = solution.joints.data();

	return solution.Success();
}

// ------------------------------------------------------------

bool RobotArmKinematicsSolver::AreValidInitializeParameters(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization ) const noexcept
{
	if ( !robot_model )
	{
		RCLCPP_ERROR( get_logger(), "Robot Model is null." );
		return false;
	}

	const auto* joint_model = robot_model->getJointModelGroup( group_name );
	if ( !joint_model )
	{
		RCLCPP_ERROR( get_logger(), "Joint model group %s not found in robot model.",
		              group_name.c_str() );
		return false;
	}

	if ( !joint_model->isChain() )
	{
		RCLCPP_ERROR( get_logger(), "Joint model group %s is not a chain.", group_name.c_str() );
		return false;
	}

	if ( !joint_model->isSingleDOFJoints() )
	{
		RCLCPP_ERROR( get_logger(), "Joint model group %s is not composed of single joints.", group_name.c_str() );
		return false;
	}

	if ( tip_frames.size() != 1 )
	{
		RCLCPP_ERROR( get_logger(), "Joint model group %s is not composed of single tip.", group_name.c_str() );
		return false;
	}

	return true;
}

// ------------------------------------------------------------

}
