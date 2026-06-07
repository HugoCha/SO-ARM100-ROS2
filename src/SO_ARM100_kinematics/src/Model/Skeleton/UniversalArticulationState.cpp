#include "Model/Skeleton/UniversalArticulationState.hpp"

#include "Global.hpp"

#include "Model/Skeleton/BoneState.hpp"
#include "UniversalSolver/UniversalModel.hpp"
#include "UniversalSolver/UniversalSolver.hpp"
#include "UniversalSolver/UniversalSolution.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

UniversalArticulationState::UniversalArticulationState( const Articulation* articulation ) :
	ArticulationState( articulation )
{
	assert( articulation->GetType() == ArticulationType::Universal );

	auto model = UniversalModel::ComputeModel(
		articulation->Joints()[0],
		articulation->Joints()[1] );

	if ( !model.has_value() )
	{
		throw std::invalid_argument( "Joints axes must intersect in one point and be independant." );
	}

	solver_ = std::make_unique< Solver::UniversalSolver >( *model, Solver::UniversalSolver::SolverParameters() );
}

// ------------------------------------------------------------

void UniversalArticulationState::ApplyConstraints( BoneState& bone_state ) const
{
	auto bone = bone_state.GetBone();

	const Vec3d& b0 = bone->Direction();
	const Vec3d& b1 = world_transform_.rotation().inverse() * bone_state.Direction();

	auto solution = solver_->SolveFromTwoVectors(
		b0,
		b1 );

	Vec3d local_dir  = solution.local_rotation * bone->Direction();
	Vec3d global_dir = world_transform_.rotation() * local_dir;
	bone_state.Direction() = global_dir.normalized() * bone->Length();
}

// ------------------------------------------------------------

void UniversalArticulationState::UpdateValues(
	const VecXd& seed,
	const BoneState& bone_state,
	double damping_factor )
{
	auto bone = bone_state.GetBone();

	const Vec3d& b0 = bone->Direction();
	const Vec3d& b1 = world_transform_.rotation().inverse() * bone_state.Direction();

	auto solution = solver_->SolveFromTwoVectors( b0, b1, seed );

	Iso3d local_transform = Iso3d::Identity();

	SetJointInternalState(
		joint_states_[0],
		local_transform,
		solution.angles[0],
		damping_factor );

	SetJointInternalState(
		joint_states_[1],
		local_transform,
		solution.angles[1],
		damping_factor );

	local_transform_ = local_transform;
	global_transform_ = world_transform_ * local_transform_;
}

// ------------------------------------------------------------

}