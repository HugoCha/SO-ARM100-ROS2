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
	double max_step{ 0.3 };
	double min_damping{ 0.1 };
	double max_damping{ 1.0 };
	double max_dq { 0.5 };
	double min_sv_tolerance{ 0.1 };
	int max_stalle_iterations{ 5 };
	double translation_weight{ 10.0 };
	double rotation_weight{ 1.0 };

	[[nodiscard]] constexpr double AdaptativeDampingCoefficient() const noexcept {
		return - std::log( 0.99 ) / ( min_sv_tolerance * min_sv_tolerance );
	}

	[[nodiscard]] constexpr bool IsValid() const noexcept {
		return max_iterations > 0 &&
		       error_tolerance > 0 &&
		       min_step > 0 && min_step <= max_step &&
		       min_damping > 0 && min_damping <= max_damping &&
			   ( translation_weight > 0 || rotation_weight > 0 ) &&
			   ( translation_weight >= 0 || rotation_weight >= 0 );
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
	double min_sv;
	double error;
	double error_reachable;
	double error_unreachable;
	double gradient;
	double step;
	double damping;
	int fk_failures;
	int stalled_error_iter;
};

SolverParameters parameters_;
mutable SolverBuffers buffers_{ 6, SolverType::Full };

[[nodiscard]] static SolverType GetSolverType( SolverParameters parameters );
[[nodiscard]] static const MatXd InitializeWeightMatrix( 
	double rotation_weight, 
	double translation_weight );

[[nodiscard]] std::optional< IterationState > InitializeState(
	const Mat4d& target, const VecXd& initial_joints ) const;

[[nodiscard]] const VecXd RandomValidJoints() const noexcept;

[[nodiscard]] const VecXd RandomValidJointsNear( 
	const VecXd& joints, 
	double distance ) const noexcept {
	assert( joints.size() == joint_chain_->GetActiveJointCount() );
	return RandomValidJointsNear( joints.data(), distance );
}

[[nodiscard]] const VecXd RandomValidJointsNear( 
	const std::span<const double>& joints, 
	double distance ) const noexcept {
	assert( joints.size() == joint_chain_->GetActiveJointCount() );
	return RandomValidJointsNear( joints.data(), distance );
}

[[nodiscard]] const VecXd RandomValidJointsNear( 
	const double* joints, 
	double distance ) const noexcept;

void PerformIteration(
	const Mat4d& target,
	IterationState& state,
	SolverBuffers& buffers ) const;

[[nodiscard]] NumericSolverState EvaluateConvergence(
	const IterationState& state,
	const MatXd& jacobian,
	const VecXd& error,
	int iteration ) const noexcept;

void UpdateDeltaQ(
	const MatXd& damped,
	const VecXd& gradient,
	VecXd& dq ) const;

void UpdateErrorConvergence(
	double last_error,
	double current_error,
	IterationState& state ) const;

[[nodiscard]] constexpr double BacktrackStep( double step ) const noexcept {
	return std::max( step * 0.5, parameters_.min_step );
}

[[nodiscard]] constexpr double BacktrackDamping( double damping ) const noexcept {
	return std::min( damping * 2.0, parameters_.max_damping );
}

[[nodiscard]] double ComputeAdaptiveStep( double min_sigma ) const noexcept {
	const double step = ( min_sigma * min_sigma ) / ( min_sigma * min_sigma + parameters_.min_sv_tolerance * parameters_.min_sv_tolerance );
	return std::clamp( step, parameters_.min_step, parameters_.max_step );
}

[[nodiscard]] double ComputeAdaptiveDamping( double min_sigma ) const noexcept {
	const double range = parameters_.max_damping - parameters_.min_damping;
	const double factor = std::exp( -parameters_.AdaptativeDampingCoefficient() * min_sigma * min_sigma );
	return parameters_.min_damping + range * factor;
}

[[nodiscard]] double GetMinSingularValue( const MatXd& jacobian ) const {
	Eigen::JacobiSVD< MatXd > svd( jacobian );
	return svd.singularValues().minCoeff();
}
};
}
