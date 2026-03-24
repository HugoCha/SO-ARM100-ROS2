#pragma once

#include "Global.hpp"

#include "Model/KinematicModel.hpp"
#include "Model/TotalLengthReachableSpace.hpp"
#include "Solver/IKProblem.hpp"

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
		std::make_unique< Model::TotalLengthReachableSpace >( chain, group.tip_home );

	return std::make_shared< const Model::KinematicModel >( 
		std::move( chain_ptr ),
		group.tip_home,
		group_topology,
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
		std::make_unique< Model::TotalLengthReachableSpace >( chain, home );

	return std::make_shared< const Model::KinematicModel >( 
		std::move( chain_ptr ),
		home,
		group_topology,
		std::move( reachable_space ) );
}

protected:
Model::KinematicModelConstPtr model_;
};
}