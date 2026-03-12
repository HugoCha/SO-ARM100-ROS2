#pragma once

#include "Global.hpp"

#include "KinematicsSolver.hpp"
#include "SolverType.hpp"
#include <Eigen/src/SVD/JacobiSVD.h>

namespace SOArm100::Kinematics
{
struct NumericSolverResult;
enum class NumericSolverState;

class DLSKinematicsSolver : public KinematicsSolver
{
public:
struct SolverParameters
{
	int max_iterations{ 200 };
	double error_tolerance{ SOArm100::Kinematics::error_tolerance };
	double gradient_tolerance { SOArm100::Kinematics::gradient_tolerance };
	double min_step{ 0.05 };
	double max_step{ 1.0 };
	double line_search_factor { 0.75 };
	double min_damping{ 5e-3 };
	double max_damping{ 1e-1 };
	double max_dq { 0.5 };
	double min_sv_tolerance{ 0.01 };
	int max_stalle_iterations{ 5 };
	double translation_weight{ 10.0 };
	double rotation_weight{ 1.0 };

	[[nodiscard]] constexpr double RotationWeightSqrt() const noexcept {
		return sqrt( rotation_weight );
	}

	[[nodiscard]] constexpr double TranslationWeightSqrt() const noexcept {
		return sqrt( translation_weight );
	}

	[[nodiscard]] constexpr double AdaptativeDampingCoefficient() const noexcept {
		return - std::log( 0.99 ) / ( min_sv_tolerance * min_sv_tolerance );
	}

	[[nodiscard]] constexpr bool IsValid() const noexcept {
		return max_iterations > 0 &&
		       error_tolerance > 0 &&
		       min_step > 0 && min_step <= max_step &&
		       min_damping > 0 && min_damping <= max_damping &&
			   ( translation_weight > 0 || rotation_weight > 0 ) &&
			   ( translation_weight >= 0 && rotation_weight >= 0 );
	}
};

public:
explicit DLSKinematicsSolver();
explicit DLSKinematicsSolver( SolverParameters parameters );
~DLSKinematicsSolver() = default;

DLSKinematicsSolver( const DLSKinematicsSolver& ) = delete;
DLSKinematicsSolver& operator = ( const DLSKinematicsSolver& ) = delete;

void SetParameters( const SolverParameters& parameters ) noexcept {
	parameters_ = parameters;
}

[[nodiscard]] const SolverParameters& GetParameters() const noexcept {
	return parameters_;
}

[[nodiscard]] NumericSolverResult InverseKinematic(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints ) const;

protected:
virtual bool InverseKinematicImpl(
	const Mat4d& target,
	const std::span< const double >& seed_joints,
	double* joints ) const override;

private:
struct SolverBuffers {
	SolverType type;

	MatXd weights;
	MatXd jacobian;
	MatXd jacobian_psi;
	MatXd weighted_jacobian;
	Eigen::JacobiSVD< MatXd > svd;
	Vec6d error;
	VecXd weighted_error;
	VecXd weighted_error_reachable;
	MatXd damped;
	VecXd gradient;
	VecXd dq;
	Mat4d fk;
	VecXd joints;
	Eigen::LDLT< MatXd > ldlt_solver;

	[[nodiscard]] inline size_t GetSize() const noexcept {
		return joints.size();
	}

	explicit SolverBuffers( size_t n_joints, SolverType solver_type ) :
		type( solver_type )
	{
		jacobian.resize( 6, n_joints );
		jacobian_psi.resize( n_joints, n_joints );
		damped.resize( n_joints, n_joints );
		joints.resize( n_joints );
		dq.resize( n_joints );
		gradient.resize( n_joints );

		if ( type == SolverType::Full )
		{
			weighted_jacobian.resize( 6, n_joints );
			weighted_error.resize( 6 );
			weighted_error_reachable.resize( 6 );
			weights.resize( 6, 6 );
		}
		else 
		{
			weighted_jacobian.resize( 3, n_joints );
			weighted_error.resize( 3 );
			weighted_error_reachable.resize( 3 );
			weights.resize( 3, 6 );
		}
	}
};

struct IterationState {
	VecXd joints;
	double error;
	double error_reachable;
	double error_unreachable;
	double gradient;
	double step;
	double damping;
	int fk_failures;
	int stalled_error_iter;
	int restart_counter;
};

enum class SeedStrategy
{
	NearCenter,
	NearJoints,
	Random,
};

struct SolverHistory
{
	int restart_counter{0};
	VecXd best_joints{0};
	double best_error{0};
};

struct SeedParameters
{
	SeedStrategy strategy{SeedStrategy::NearJoints};
	double distance{0.05};
	double margin_percent{0.1};
};

SolverParameters parameters_;
mutable SolverBuffers buffers_{ 6, SolverType::Full };
mutable random_numbers::RandomNumberGenerator rng_;

[[nodiscard]] static SolverType GetSolverType( SolverParameters parameters );
[[nodiscard]] static const MatXd InitializeWeightMatrix( 
	double rotation_weight, 
	double translation_weight );

[[nodiscard]] std::optional< IterationState > InitializeState(
	const Mat4d& target, const VecXd& initial_joints ) const;

[[nodiscard]] SolverHistory InitializeHistory( 
	const std::optional< IterationState >& state, const VecXd& initial_joints ) const;

[[nodiscard]] SeedParameters InitializeSeedParameters(
	const std::optional< IterationState >& state, const VecXd& initial_joints ) const;

[[nodiscard]] const VecXd RandomValidJoints( double margin_percent ) const noexcept;

[[nodiscard]] const VecXd RandomValidJointsTargeted( 
	const VecXd& joints, 
	double distance,
	double margin_percent ) const noexcept {
	assert( joints.size() == joint_chain_->GetActiveJointCount() );
	return RandomValidJointsTargeted( joints.data(), distance, margin_percent );
}

[[nodiscard]] const VecXd RandomValidJointsTargeted( 
	const std::span<const double>& joints, 
	double distance,
	double margin_percent ) const noexcept {
	assert( joints.size() == joint_chain_->GetActiveJointCount() );
	return RandomValidJointsTargeted( joints.data(), distance, margin_percent );
}

[[nodiscard]] const VecXd RandomValidJointsTargeted( 
	const double* joints, 
	double distance,
	double margin_percent ) const noexcept;

[[nodiscard]] const VecXd RandomValidJointsNear( 
	const VecXd& joints, 
	double distance,
	double margin_percent ) const noexcept {
	assert( joints.size() == joint_chain_->GetActiveJointCount() );
	return RandomValidJointsNear( joints.data(), distance, margin_percent );
}

[[nodiscard]] const VecXd RandomValidJointsNear( 
	const std::span<const double>& joints, 
	double distance,
	double margin_percent ) const noexcept {
	assert( joints.size() == joint_chain_->GetActiveJointCount() );
	return RandomValidJointsNear( joints.data(), distance, margin_percent );
}

[[nodiscard]] const VecXd RandomValidJointsNear( 
	const double* joints, 
	double distance,
	double margin_percent ) const noexcept;

constexpr double SmallError() const noexcept {
	return parameters_.rotation_weight * small_rotation_error + 
		   parameters_.translation_weight * small_translation_error;
}

constexpr double LargeError() const noexcept {
	return parameters_.rotation_weight * large_rotation_error + 
		   parameters_.translation_weight * large_translation_error;
}

void PerformIteration(
	const Mat4d& target,
	IterationState& state,
	SolverBuffers& buffers ) const;

void UpdateSeedJoints(
	const SeedParameters& seed_parameters,
	const SolverHistory& history,
	VecXd& seed_joints ) const;

bool UpdateBuffer(
	const Mat4d& target,
	const VecXd& joints,
	SolverBuffers& buffers ) const;

void UpdateHistory(
	const IterationState& state,
	SolverHistory& history ) const;

void LineSearch(
	const Mat4d& target,
	IterationState& state,
	SolverBuffers& buffers ) const;

void JacobianPSI( const Eigen::JacobiSVD< MatXd >& svd, MatXd& jacobian_psi ) const;

void PrintIteration( const IterationState& state, const SolverBuffers& buffers ) const;
void PrintState( const IterationState& state ) const;
void PrintBuffer( const SolverBuffers& buffers ) const;

[[nodiscard]] NumericSolverState EvaluateConvergence(
	const IterationState& state,
	int iteration ) const noexcept;

void UpdateDeltaQPrimary(
	const MatXd& damped,
	const VecXd& gradient,
	VecXd& dq_primary ) const;

void UpdateDeltaQSecondary(
	const VecXd& joints,
	const MatXd& jacobian,
	const MatXd& jacobian_psi,
	VecXd& dq_secondary ) const;

void UpdateDeltaQ(
	const MatXd& damped,
	const VecXd& gradient,
	const VecXd& joints,
	const MatXd& jacobian,
	const MatXd& jacobian_psi,
	VecXd& dq ) const;

void ClampDeltaQ( const VecXd& joints, VecXd& dq ) const;

void UpdateErrorConvergence(
	double last_error,
	double current_error,
	IterationState& state ) const;

[[nodiscard]] SeedStrategy ChooseSeedStategy(
	const IterationState& state,
	const SolverHistory& history ) const;

void UpdateSeedParameters(
	const IterationState& state,
	const SolverHistory& history,
	SeedParameters& seed_parameters
) const;

};
}
