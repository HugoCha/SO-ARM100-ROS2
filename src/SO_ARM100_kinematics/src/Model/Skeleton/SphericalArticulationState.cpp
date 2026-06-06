#include "Model/Skeleton/SphericalArticulationState.hpp"

#include "Global.hpp"

#include "SphericalSolver/SphericalModel.hpp"
#include "SphericalSolver/SphericalSolver.hpp"
#include "SphericalSolver/SphericalSolution.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

SphericalArticulationState::SphericalArticulationState( const Articulation* articulation ) :
	ArticulationState( articulation ),
	solver_( nullptr )
{
	assert( articulation->GetType() == ArticulationType::Spherical );

	auto model = SphericalModel::ComputeModel(
		articulation->Joints()[0],
		articulation->Joints()[1],
		articulation->Joints()[2] );

	if ( !model.has_value() )
	{
		throw std::invalid_argument( "Joints axes must intersect in one point and be independant." );
	}
	
	solver_ = std::make_unique< Solver::SphericalSolver >( 
		*model, 
		Solver::SphericalSolver::SolverParameters() );
}

// ------------------------------------------------------------

void SphericalArticulationState::ApplyConstraints( BoneState& bone_state ) const
{
	const auto& bone = bone_state.GetBone();
	Vec3d old_bone = bone->Direction();
	Vec3d new_bone = world_transform_.rotation().inverse() * bone_state.Direction();

	auto result = solver_->SolveFromTwoVectors( old_bone, new_bone );
	
	if ( !result.reachable )
	{
		auto local_rotation = 
			AngleAxis( result.angles[2], articulation_->Joints()[2]->Axis() ) * 
			AngleAxis( result.angles[1], articulation_->Joints()[1]->Axis() ) * 
			AngleAxis( result.angles[0], articulation_->Joints()[0]->Axis() );
		
		Vec3d bone_dir = world_transform_.rotation() * local_rotation * bone->Direction();
		bone_state.Direction() = bone_dir.normalized() * bone->Length();
	}
	else
	{
		bone_state.Direction() = bone_state.Direction().normalized() * bone->Length();
	}
}

// ------------------------------------------------------------

void SphericalArticulationState::UpdateValues( 
	const VecXd& seed,
    const BoneState& bone_state, 
    double damping_factor )
{
	Vec3d angles;

	Vec3d old_bone = bone_state.GetBone()->Direction();
	Vec3d new_bone = world_transform_.rotation().inverse() * bone_state.Direction();

	auto result = solver_->SolveAndOptimizeFromTwoVectors(
		old_bone,
		new_bone,
		seed );

	angles = result.angles;

	Iso3d local_transform = Iso3d::Identity();

	SetJointInternalState(
		joint_states_[0],
		local_transform,
		angles[0],
		damping_factor );

	SetJointInternalState(
		joint_states_[1],
		local_transform,
		angles[1],
		damping_factor );

	SetJointInternalState(
		joint_states_[2],
		local_transform,
		angles[2],
		damping_factor );

	local_transform_ = local_transform;
	global_transform_ = world_transform_ * local_transform_;
}

// ------------------------------------------------------------

}