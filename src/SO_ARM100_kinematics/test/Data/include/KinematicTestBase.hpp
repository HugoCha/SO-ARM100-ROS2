#pragma once

#include "Global.hpp"

#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/JointChainBuilder.hpp"
#include "Model/Joint/JointType.hpp"
#include "Model/KinematicModel.hpp"
#include "Model/ReachableSpace/ChainTotalLengthReachableSpace.hpp"
#include "ModelAnalyzer/SkeletonAnalyzer.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKSolution.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <gtest/gtest.h>
#include <stdexcept>

namespace SOArm100::Kinematics::Test
{
class KinematicTestBase : public ::testing::Test
{
public:
Mat4d ComputeFK(
	Model::KinematicModelConstPtr model,
	const VecXd& joints ) const
{
	Mat4d fk;
	model->ComputeFK( joints, fk );
	return fk;
}

Solver::IKProblem CreateProblem(
	Model::KinematicModelConstPtr model,
	const VecXd& seed,
	const VecXd& joints,
	double tolerance =  SOArm100::Kinematics::error_tolerance ) const
{
	Mat4d fk;
	if ( !model->ComputeFK( joints, fk ) )
		throw std::invalid_argument( "Impossible to create problem, check limits" );
	return CreateProblem( seed, fk, tolerance );
}

Solver::IKProblem CreateProblem(
	VecXd seed,
	Mat4d target,
	double tolerance =  SOArm100::Kinematics::error_tolerance ) const
{
	return {
	    target,
	    seed,
	    VecXd::Zero( seed.size() ),
	    tolerance,
	    100 };
}

double PoseError(
	Model::KinematicModelConstPtr model,
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const
{
	Vec6d pose_error;
	SOArm100::Kinematics::PoseError(
		problem.target,
		ComputeFK( model, solution.joints ),
		pose_error );
	return pose_error.norm();
};

double PoseError(
	Model::KinematicModelConstPtr model,
	const Solver::IKProblem& problem,
	const Heuristic::IKPresolution& presolution ) const
{
	Vec6d pose_error;
	SOArm100::Kinematics::PoseError(
		problem.target,
		ComputeFK( model, presolution.joints ),
		pose_error );
	return pose_error.norm();
};

double PositionError(
	Model::KinematicModelConstPtr model,
	const Solver::IKProblem& problem,
	const Solver::IKSolution& solution ) const
{
	return TranslationError(
		problem.target,
		ComputeFK( model, solution.joints ) );
};

Model::KinematicModelConstPtr CreateModel(
	Model::JointChainConstPtr chain,
	const Model::JointGroup& group ) const
{
	Model::KinematicTopology group_topology;
	group_topology.Add( group );

	auto reachable_space =
		std::make_unique< Model::ChainTotalLengthReachableSpace >( *chain, group.tip_home );

	return std::make_shared< const Model::KinematicModel >(
		std::move( chain ),
		group.tip_home,
		group_topology,
		std::move( Model::SkeletonAnalyzer::Analyze( chain->GetJoints(), group ) ),
		std::move( reachable_space ) );
}

Model::KinematicModelConstPtr CreateModel(
	Model::JointChainConstPtr chain,
	const Mat4d& home ) const
{
	Model::KinematicTopology group_topology;

	auto reachable_space =
		std::make_unique< Model::ChainTotalLengthReachableSpace >( *chain, home );

	return std::make_shared< const Model::KinematicModel >(
		std::move( chain ),
		home,
		group_topology,
		std::move( Model::SkeletonAnalyzer::Analyze( chain->GetJoints(), home ) ),
		std::move( reachable_space ) );
}

Model::JointConstPtr MakeRevoluteJoint(
	const Vec3d& axis,
	const Vec3d& origin,
	double min   = -M_PI / 2,
	double max   =  M_PI / 2 ) const
{
	return std::make_shared< const Model::Joint >(
		"",
		ToTransformMatrix( origin ),
		Model::Twist( axis, origin ),
		Model::Limits( min, max ),
		nullptr,
		nullptr );
}

Model::JointConstPtr MakePrismaticJoint(
	const Vec3d& axis,
	const Vec3d& origin,
	double min = 0.0,
	double max = 1.0 ) const
{
	return std::make_shared< const Model::Joint >(
		"",
		ToTransformMatrix( origin ),
		Model::Twist( axis ),
		Model::Limits( min, max ),
		nullptr,
		nullptr );
}

Model::JointChainBuilder& AddBaseLink( Model::JointChainBuilder& builder )
{
	builder.AddParentLink( "base", Mat4d::Identity() );
	return builder;
}

Model::JointChainBuilder& AddRevoluteJointLink(
	Model::JointChainBuilder& builder, 
	int index,
	const Mat4d& joint_home,
	const Vec3d& joint_axis,
	const Mat4d& link_home,
	double min = -M_PI, 
	double max = M_PI )
{
	builder.AddJoint( "r_joint" + std::to_string( index ), 
					  joint_home, 
					  { joint_axis, Translation( joint_home ) }, 
					  { min, max } );

	builder.AddChildLink( "link" + std::to_string( index ), 
						  link_home, 
						  link_home * Inverse( joint_home ) );
	
	return builder;
}

Model::JointChainBuilder& AddPrismaticJointLink(
	Model::JointChainBuilder& builder, 
	int index,
	const Mat4d& joint_home,
	const Vec3d& joint_axis,
	const Mat4d& link_home,
	double min = 0, 
	double max = 1 )
{
	builder.AddJoint( "p_joint" + std::to_string( index ), 
					  joint_home, 
					  { joint_axis }, 
					  { min, max } );

	builder.AddChildLink( "link" + std::to_string( index ), 
						  link_home, 
						  link_home * Inverse( joint_home ) );
	
	return builder;
}

Model::JointChainBuilder& AddJointTipLink(
	Model::JointChainBuilder& builder, 
	int index,
	const Model::JointType type,
	const Mat4d& joint_home,
	const Vec3d& joint_axis,
	const Mat4d& tip_home,
	double min = -M_PI, 
	double max = M_PI )
{
	Model::Twist t;
	if ( type == Model::JointType::FIXED )
		t = Model::Twist();
	else if ( type == Model::JointType::PRISMATIC )
		t = Model::Twist( joint_axis );
	else
		t = Model::Twist( joint_axis, Translation( joint_home ) );

	Model::Limits l = type == Model::JointType::FIXED ? Model::Limits{} : Model::Limits{ min, max };

	builder.AddJoint( "joint" + std::to_string( index ), 
					  joint_home, 
					  t, 
					  l );

	builder.AddChildLink( "link_tip", 
						  tip_home, 
						  tip_home * Inverse( joint_home ) );
	
	return builder;
}

struct JointInfo 
{
	Vec3d origin;
	Vec3d axis;
	Model::JointType type;
	double min;
	double max;
};

struct RevoluteJointInfo : JointInfo
{
	RevoluteJointInfo( Vec3d origin, Vec3d axis, double min = -M_PI, double max = M_PI ) :
		JointInfo( origin, axis, Model::JointType::REVOLUTE, min, max )
	{}
};

struct PrismaticJointInfo : JointInfo
{
	PrismaticJointInfo( Vec3d origin, Vec3d axis, double min = 0, double max = 1 ) :
		JointInfo( origin, axis, Model::JointType::PRISMATIC, min, max )
	{}
};

Model::JointChainConstPtr CreateSimpleJointChain( const std::vector< JointInfo > infos, const Mat4d& tip_home )
{
	if ( infos.empty() ) return nullptr;

	auto builder = Model::JointChainBuilder();

	AddBaseLink( builder );

	for ( int i = 0; i < infos.size() - 1; i++ )
	{
		if ( infos[i].type == Model::JointType::REVOLUTE )
			AddRevoluteJointLink( builder, i, ToTransformMatrix( infos[i].origin ), infos[i].axis, ToTransformMatrix( infos[i+1].origin ), infos[i].min, infos[i].max );
		else if ( infos[i].type == Model::JointType::PRISMATIC )
			AddRevoluteJointLink( builder, i, ToTransformMatrix( infos[i].origin ), infos[i].axis, ToTransformMatrix( infos[i+1].origin ), infos[i].min, infos[i].max );
	}

	AddJointTipLink( builder, infos.size(), infos[infos.size()-1].type, ToTransformMatrix( infos[infos.size()-1].origin ), infos[infos.size()-1].axis, tip_home, infos[infos.size()-1].min, infos[infos.size()-1].max );

	return builder.Build();
}

protected:
Model::KinematicModelConstPtr model_;
};
}