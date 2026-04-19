#include "KinematicsSolver.hpp"

#include "Global.hpp"
#include "HybridSolver/HybridSolver.hpp"
#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/Twist.hpp"
#include "Model/KinematicModel.hpp"
#include "Model/KinematicTopology/KinematicTopology.hpp"
#include "Model/ReachableSpace/SkeletonTotalLengthReachableSpace.hpp"
#include "ModelAnalyzer/SkeletonAnalyzer.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKRunContext.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

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
#include <vector>

namespace SOArm100::Kinematics
{

// ------------------------------------------------------------

static rclcpp::Logger get_logger()
{
	static rclcpp::Logger logger = rclcpp::get_logger( "KinematicsSolver" );
	return logger;
}

// ------------------------------------------------------------

KinematicsSolver::KinematicsSolver() :
	model_( nullptr )
{
}

// ------------------------------------------------------------

void KinematicsSolver::Initialize(
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const std::string& base_frame,
	const std::vector< std::string >& tip_frames,
	double search_discretization )
{
	RCLCPP_INFO( get_logger(), "Initializing KinematicsSolver for group: %s",
	             group_name.c_str() );

	if ( !AreValidInitializeParameters( robot_model, group_name, base_frame, tip_frames, search_discretization ) )
	{
		return;
	}

	moveit::core::RobotState state( robot_model );
	state.setToDefaultValues();

	const auto* joint_model_group = robot_model->getJointModelGroup( group_name );
	const auto& link_models = joint_model_group->getLinkModels();
	const int n_joints = joint_model_group->getJointModels().size();

	auto joint_chain = std::make_shared< Model::JointChain >( n_joints );

	for ( const auto* link_model : link_models )
	{
		const auto* parent_joint = link_model->getParentJointModel();
		const auto& child_joints = link_model->getChildJointModels();

		const Iso3d& joint_transform = state.getGlobalLinkTransform( link_model );
		Iso3d child_transform = joint_transform;
		if ( !child_joints.empty() )
		{
			child_transform = state.getGlobalLinkTransform( child_joints[0]->getChildLinkModel() );
		}
		Model::Link link( joint_transform.matrix(), child_transform.matrix() );

		Model::Limits limits;
		const auto& bounds = parent_joint->getVariableBounds();
		if ( !bounds.empty() )
		{
			const auto& lim = bounds[0];
			limits = Model::Limits( lim.min_position_, lim.max_position_ );
		}

		Model::Twist twist;
		if ( parent_joint->getType() == moveit::core::JointModel::REVOLUTE )
		{
			const auto* revolute_joint_model =
				static_cast< const moveit::core::RevoluteJointModel* >( parent_joint );

			const Vec3d axis_world = joint_transform.rotation() * revolute_joint_model->getAxis();
			const Vec3d point_on_axis_world = joint_transform.translation();

			twist = Model::Twist( axis_world, point_on_axis_world );
		}
		else if ( parent_joint->getType() == moveit::core::JointModel::PRISMATIC )
		{
			const auto* prismatic_joint_model =
				static_cast< const moveit::core::PrismaticJointModel* >( parent_joint );

			twist = Model::Twist( prismatic_joint_model->getAxis() );
		}

		joint_chain->Add( twist, link, limits );
	}

	auto joint_chain_const = std::make_shared< const Model::JointChain >( *joint_chain );
	const Mat4d& home_configuration = state.getGlobalLinkTransform( tip_frames[0] ).matrix();
	auto skeleton = Model::SkeletonAnalyzer::Analyze( joint_chain->GetJoints(), home_configuration );
	std::unique_ptr< const Model::ReachableSpace > reachable_space =
		std::make_unique< const Model::SkeletonTotalLengthReachableSpace >( *skeleton );

	Model::KinematicTopology topology;

	model_ = std::make_shared< const Model::KinematicModel >(
		joint_chain_const,
		home_configuration,
		topology,
		skeleton,
		std::move( reachable_space ) );
}

// ------------------------------------------------------------

void KinematicsSolver::Initialize(
	Model::KinematicModelConstPtr model,
	const Solver::HybridSolver& solver,
	double search_discretization )
{
	model_ = model;
}

// ------------------------------------------------------------

bool KinematicsSolver::ForwardKinematic(
	const std::span< const double >& joints,
	geometry_msgs::msg::Pose& pose ) const
{
	Mat4d T_end;
	if ( ForwardKinematic( ToVecXd( joints ), T_end ) )
	{
		pose = ToPoseMsg( T_end );
		return true;
	}
	return false;
}

// ------------------------------------------------------------

bool KinematicsSolver::ForwardKinematic(
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

bool KinematicsSolver::InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	const std::span< const double >& seed_joints,
	std::vector< double >& joints ) const
{
	if ( !model_ || !solver_ )
	{
		RCLCPP_ERROR( get_logger(), "Solver or Joint model not initialized." );
		return false;
	}

	const int n_joints = model_->GetChain()->GetActiveJointCount();
	if ( joints.size() != n_joints )
		joints.resize( n_joints );

	return InverseKinematic(
		ToTransformMatrix( target_pose ),
		ToVecXd( seed_joints ),
		joints.data(),
		n_joints );
}

// ------------------------------------------------------------

bool KinematicsSolver::InverseKinematic(
	const Mat4d& target,
	const VecXd& seed,
	VecXd& joints ) const
{
	if ( !model_ || !solver_ )
	{
		RCLCPP_ERROR( get_logger(), "Solver or Joint model not initialized." );
		return false;
	}

	const int n_joints = model_->GetChain()->GetActiveJointCount();
	if ( joints.size() != n_joints )
		joints.resize( n_joints );

	return InverseKinematic(
		target,
		seed,
		joints.data(),
		n_joints );
}

// ------------------------------------------------------------

bool KinematicsSolver::InverseKinematic(
	const Mat4d& target,
	const VecXd& seed,
	double* joints,
	int n_joints ) const
{
	if ( seed.size() != n_joints )
	{
		RCLCPP_ERROR( get_logger(), "Wrong seed size." );
		return false;
	}

	Solver::IKProblem problem {
		target,
		seed,
		translation_tolerance,
		rotation_tolerance,
		timeout };

	Solver::IKRunContext context;

	auto solution = solver_->Solve( problem, context );
	joints = solution.joints.data();

	return solution.Success();
}

// ------------------------------------------------------------

bool KinematicsSolver::AreValidInitializeParameters(
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
