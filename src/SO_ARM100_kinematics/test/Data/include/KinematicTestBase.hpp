#pragma once

#include "Global.hpp"

#include "Model/KinematicModel.hpp"
#include "Model/ReachableSpace/ChainTotalLengthReachableSpace.hpp"
#include "ModelAnalyzer/SkeletonAnalyzer.hpp"
#include "Solver/IKProblem.hpp"
#include "Utils/Converter.hpp"

#include <gtest/gtest.h>

namespace SOArm100::Kinematics::Test
{
class KinematicTestBase : public ::testing::Test
{
public:
Mat4d ComputeFK( const VecXd& joints )
{
	Mat4d fk;
	model_->ComputeFK( joints, fk );
	return fk;
}

Solver::IKProblem CreateProblem(
	Model::KinematicModelConstPtr model,
	const VecXd& seed,
	const VecXd& joints )
{
	Mat4d fk;
	model->ComputeFK( joints, fk );
	return CreateProblem( seed, fk );
}

Solver::IKProblem CreateProblem( const VecXd& seed, const VecXd& joints )
{
	return CreateProblem( seed, ComputeFK( joints ) );
}

Solver::IKProblem CreateProblem( const VecXd& seed, const Mat4d& target  )
{
	return {
	    target,
	    seed,
	    translation_tolerance,
	    rotation_tolerance,
	    100 };
}

Model::KinematicModelConstPtr CreateModel(
	const Model::JointChain& chain,
	const Model::JointGroup& group )
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
	const Mat4d& home )
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
	double max   =  M_PI / 2 )
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
	double max = 1.0 )
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