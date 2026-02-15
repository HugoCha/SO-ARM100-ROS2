#pragma once

#include "Global.hpp"

#include "KinematicsSolver.hpp"

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
	double error_tolerance{ 1e-6 };
	double min_step{ 0.1 };
	double max_step{ 1.0 };
	double min_damping{ 0.01 };
	double max_damping{ 0.1 };
	double epsilon_step{ 0.01 };
	double min_sv_factor{ 10.0 };
	int max_stalle_iterations{ 5 };
	double translation_weight{ 10.0 };
	double rotation_weight{ 1.0 };

	[[nodiscard]] constexpr bool IsValid() const noexcept {
		return max_iterations > 0 &&
		       error_tolerance > 0 &&
		       min_step > 0 && min_step <= max_step &&
		       min_damping > 0 && min_damping <= max_damping &&
		       translation_weight > 0 && rotation_weight > 0;
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

virtual bool InverseKinematic(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints,
	VecXd& joint_angles ) const override;

[[nodiscard]] NumericSolverResult SolveIK(
	const Mat4d& target_pose,
	const std::span< const double >& seed_joints ) const;

private:
struct SolverBuffers {
	MatXd jacobian;
	MatXd damped;
	MatXd jac_transpose;
	VecXd dq;
	Vec6d error;
	Mat4d fk;
	VecXd joints;
	Eigen::LDLT< MatXd > ldlt_solver;

	[[nodiscard]] inline size_t GetSize() const noexcept {
		return dq.size();
	}

	explicit SolverBuffers( size_t n_joints )
	{
		jacobian.resize( 6, n_joints );
		damped.resize( n_joints, n_joints );
		jac_transpose.resize( n_joints, 6 );
		dq.resize( n_joints );
	}
};

struct IterationState {
	VecXd joints;
	double error;
	double step;
	double damping;
	int fk_failures;
	int stalled_error_iter;
};

SolverParameters parameters_;
mutable SolverBuffers buffers_{ 6 };

[[nodiscard]] std::optional< IterationState > InitializeState(
	const Mat4d& target, const VecXd& initial_joints ) const;
[[nodiscard]] const Mat6d InitializeWeightMatrix() const;

[[nodiscard]] const VecXd RandomValidJoints() const noexcept;

void PerformIteration(
	const Mat4d& target,
	const Mat6d& weights,
	IterationState& state,
	SolverBuffers& buffers ) const;

[[nodiscard]] NumericSolverState EvaluateConvergence(
	const IterationState& state,
	int iteration ) const noexcept;

void ComputeWeightedJacobianAndDamping(
	const VecXd& joints,
	const Mat6d& weights,
	double damping_factor,
	SolverBuffers& buffers ) const;

void UpdateDeltaQ(
	const MatXd& jacobian,
	const VecXd& error,
	const MatXd& damped,
	VecXd& dq_out ) const;

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
	const double step = ( min_sigma * min_sigma ) / ( min_sigma * min_sigma + parameters_.epsilon_step );
	return std::clamp( step, parameters_.min_step, parameters_.max_step );
}

[[nodiscard]] double ComputeAdaptiveDamping( double min_sigma ) const noexcept {
	const double range = parameters_.max_damping - parameters_.min_damping;
	const double factor = std::exp( -parameters_.min_sv_factor * min_sigma * min_sigma );
	return parameters_.min_damping + range * factor;
}

[[nodiscard]] double GetMinSingularValue( const MatXd& jacobian ) const {
	Eigen::JacobiSVD< MatXd > svd( jacobian );
	return std::max( svd.singularValues().minCoeff(), 1e-6 );
}

[[nodiscard]] bool IsStalled( const IterationState& state ) const noexcept {
	return ( state.step <= parameters_.min_step &&
	         state.damping >= parameters_.max_damping ) ||
	       ( state.stalled_error_iter >= parameters_.max_stalle_iterations ) ||
	       ( state.fk_failures >= parameters_.max_stalle_iterations );
}
};
}
