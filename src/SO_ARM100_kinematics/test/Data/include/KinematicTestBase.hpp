#pragma once

#include "Global.hpp"

#include "Model/KinematicModel.hpp"
#include "Model/ReachableSpace/ChainTotalLengthReachableSpace.hpp"
#include "ModelAnalyzer/SkeletonAnalyzer.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKSolution.hpp"
#include "Utils/Converter.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <gtest/gtest.h>

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
	const VecXd& joints ) const
{
	Mat4d fk;
	model->ComputeFK( joints, fk );
	return CreateProblem( seed, fk );
}

Solver::IKProblem CreateProblem( const VecXd& seed, Mat4d target  ) const
{
	return {
	    target,
	    seed,
	    translation_tolerance,
	    rotation_tolerance,
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
	const Model::JointChain& chain,
	const Model::JointGroup& group ) const
{
	auto chain_ptr =
		std::make_unique< const Model::JointChain >( chain );

	Model::KinematicTopology group_topology;
	group_topology.Add( group );

	auto reachable_space =
		std::make_unique< Model::ChainTotalLengthReachableSpace >( chain, group.tip_home );

	return std::make_shared< const Model::KinematicModel >(
		std::move( chain_ptr ),
		group.tip_home,
		group_topology,
		std::move( Model::SkeletonAnalyzer::Analyze( chain.GetJoints(), group ) ),
		std::move( reachable_space ) );
}

Model::KinematicModelConstPtr CreateModel(
	const Model::JointChain& chain,
	const Mat4d& home ) const
{
	auto chain_ptr =
		std::make_unique< const Model::JointChain >( chain );

	Model::KinematicTopology group_topology;

	auto reachable_space =
		std::make_unique< Model::ChainTotalLengthReachableSpace >( chain, home );

	return std::make_shared< const Model::KinematicModel >(
		std::move( chain_ptr ),
		home,
		group_topology,
		std::move( Model::SkeletonAnalyzer::Analyze( chain.GetJoints(), home ) ),
		std::move( reachable_space ) );
}

Model::JointConstPtr MakeRevoluteJoint(
	const Vec3d& axis,
	const Vec3d& origin,
	double min   = -M_PI / 2,
	double max   =  M_PI / 2 ) const
{
	return std::make_shared< const Model::Joint >(
		Model::Twist( axis, origin ),
		Model::Link( ToTransformMatrix( origin ), 0 ),
		Model::Limits( min, max )
		);
}

Model::JointConstPtr MakePrismaticJoint(
	const Vec3d& axis,
	const Vec3d& origin,
	double min = 0.0,
	double max = 1.0 ) const
{
	return std::make_shared< const Model::Joint >(
		Model::Twist( axis ),
		Model::Link( ToTransformMatrix( origin ), 0 ),
		Model::Limits( min, max ) );
}

protected:
Model::KinematicModelConstPtr model_;
};
}