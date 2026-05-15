#include "Model/Skeleton/SphericalArticulationState.hpp"

#include "Global.hpp"

#include "Euler/EulerModel.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

SphericalArticulationState::SphericalArticulationState( const Articulation* articulation ) :
	ArticulationState( articulation )
{
	assert( articulation->GetType() == ArticulationType::Spherical );

	auto euler_model = EulerModel::ComputeModel(
		articulation->Joints()[0],
		articulation->Joints()[1],
		articulation->Joints()[2] );

	assert( euler_model.has_value() );

	spherical_solver_ = std::make_unique< Solver::SphericalSolver >( *euler_model, Solver::SphericalSolver::SolverParameters() );
}

// ------------------------------------------------------------

void SphericalArticulationState::ApplyConstraints( BoneState& bone_state ) const
{
	if ( bone_state.Direction().norm() != bone_state.GetBone()->Length() )
		bone_state.Direction() = bone_state.GetBone()->Length() * bone_state.Direction().normalized();
}

// ------------------------------------------------------------

void SphericalArticulationState::UpdateValues( const BoneState& bone_state )
{
	assert( spherical_solver_ );

	Vec3d old_bone = bone_state.GetBone()->Direction();
	Vec3d new_bone = world_transform_.rotation().inverse() * bone_state.Direction();
	
	auto result = spherical_solver_->Solve(
		old_bone, 
		new_bone );
	Vec3d angles = result.angles;

	local_transform_.setIdentity();
	SetJointInternalState( 
		joint_states_[0], 
		world_transform_, 
		global_transform_, 
		local_transform_, 
		angles[0] );

	SetJointInternalState( 
		joint_states_[1], 
		world_transform_, 
		global_transform_, 
		local_transform_, 
		angles[1] );

	SetJointInternalState( 
		joint_states_[2], 
		world_transform_, 
		global_transform_, 
		local_transform_, 
		angles[2] );
}

// ------------------------------------------------------------

}