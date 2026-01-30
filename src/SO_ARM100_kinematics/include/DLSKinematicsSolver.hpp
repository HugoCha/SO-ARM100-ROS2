#pragma once

#include "KinematicsSolver.hpp"
#include "Types.hpp"

#include <Eigen/Geometry>

namespace SOArm100::Kinematics
{
class DLSKinematicsSolver : public KinematicsSolver
{
public:
struct SolverParameters
{
	int max_iterations{ 100 };
	double error_tolerance{ 1e-4 };
	double min_step{ 0.01 };
	double max_step{ 1.0 };
	double min_damping{ 0.001 };
	double max_damping{ 1.0 };
	double epsilon_step{ 0.1 };
	double min_sv_factor{ 10.0 };
	int max_stale_iterations{ 5 };

	[[nodiscard]] constexpr bool IsValid() const noexcept {
		return max_iterations > 0 &&
		       error_tolerance > 0 &&
		       min_step > 0 && min_step <= max_step &&
		       min_damping > 0 && min_damping <= max_damping;
	}
};

enum class SolverState
{
	Converged,
	Improving,
	Stalled,
	MaxIterations,
	Failed
};

struct IKResult
{
	SolverState state;
	VecXd joint_angles;
	double final_error;
	int iterations_used;

	[[nodiscard]] bool Success() const noexcept {
		return state == SolverState::Converged;
	}
};

explicit DLSKinematicsSolver();
explicit DLSKinematicsSolver( SolverParameters parameters );
~DLSKinematicsSolver() = default;

DLSKinematicsSolver( const DLSKinematicsSolver& ) = delete;
DLSKinematicsSolver& operator = ( const DLSKinematicsSolver& ) = delete;

DLSKinematicsSolver( DLSKinematicsSolver&& ) noexcept = default;
DLSKinematicsSolver& operator = ( DLSKinematicsSolver&& ) noexcept = default;

[[nodiscard]] IKResult SolveIK( const geometry_msgs::msg::Pose& target_pose );

void SetConfig( const SolverParameters& parameters ) noexcept {
	parameters_ = parameters;
}

[[nodiscard]] const SolverParameters& GetParameters() const noexcept {
	return parameters_;
}

virtual bool InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	std::vector< double >& joint_angles ) override;

private:
struct SolverBuffers {
	MatXd jacobian;
	MatXd damped;
	MatXd jac_transpose;
	VecXd dq;
	VecXd error;
	Mat4d fk;
	Eigen::LDLT< MatXd > ldlt_solver;

	explicit SolverBuffers( size_t n_joints )
	{
		jacobian.resize( 6, n_joints );
		damped.resize( n_joints, n_joints );
		jac_transpose.resize( n_joints, 6 );
		dq.resize( n_joints );
		error.resize( 6 );
	}
};

struct IterationState {
	VecXd joints;
	double error;
	double step;
	double damping;
	int fk_failures;
};

SolverParameters parameters_;
mutable SolverBuffers buffers_{ 6 };  // Taille par défaut

[[nodiscard]] std::optional< IterationState > InitializeState(
	const Mat4d& target ) const;

void PerformIteration(
	const Mat4d& target,
	IterationState& state,
	SolverBuffers& buffers );

[[nodiscard]] SolverState EvaluateConvergence(
	const IterationState& state,
	int iteration ) const noexcept;

void ComputeJacobianAndDamping(
	const VecXd& joints,
	double damping_factor,
	SolverBuffers& buffers ) const;

void UpdateDeltaQ(
	const MatXd& jacobian,
	const VecXd& error,
	const MatXd& damped,
	VecXd& dq_out ) const;

VecXd InitializeJointAngles();
VecXd RandomValidJointAngles();
bool Initialize(
	const Mat4d& target,
	const VecXd& initial_joints,
	Mat4d& fk,
	MatXd& jacobian,
	VecXd& pose_error,
	double& error,
	double& damping_factor,
	double& step,
	MatXd& damped,
	VecXd& dq );

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
	       ( state.fk_failures >= parameters_.max_stale_iterations );
}
};
}
