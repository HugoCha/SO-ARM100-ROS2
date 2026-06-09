#pragma once

#include "Global.hpp"

#include "DLSSolverParameters.hpp"
#include "Model/IKModelBase.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKSolverBase.hpp"

#include <Eigen/src/SVD/JacobiSVD.h>


namespace SOArm100::Kinematics::Model
{
enum class RandomType;
}

namespace SOArm100::Kinematics::Seed
{
class IKRandomSeedGenerator;
}

namespace SOArm100::Kinematics::Solver
{
enum class DLSSolverState;
enum class SolverType;
struct SolverHistory;

class DLSSolver : public Model::IKModelBase, public IKSolverBase
{
public:
explicit DLSSolver( Model::KinematicModelConstPtr model );
explicit DLSSolver(
	Model::KinematicModelConstPtr model,
	DLSSolverParameters parameters );
~DLSSolver() = default;

DLSSolver( const DLSSolver& ) = delete;
DLSSolver& operator = ( const DLSSolver& ) = delete;

void SetParameters( const DLSSolverParameters& parameters ) noexcept {
	parameters_ = parameters;
}

[[nodiscard]] const DLSSolverParameters& GetParameters() const noexcept {
	return parameters_;
}

virtual IKSolution Solve(
	const IKProblem& problem,
	const IKRunContext& context ) const override;

private:
struct SolverBuffers;
struct IterationState;

DLSSolverParameters parameters_;

[[nodiscard]] static SolverType GetSolverType( const DLSSolverParameters& parameters );
[[nodiscard]] static const MatXd InitializeWeightMatrix(
	double rotation_weight,
	double translation_weight );

[[nodiscard]] std::optional< IterationState > InitializeState(
	const Mat4d& target, const VecXd& initial_joints, SolverBuffers& buffers ) const;

[[nodiscard]] SolverHistory InitializeHistory(
	const std::optional< IterationState >& state, const VecXd& initial_joints ) const;

[[nodiscard]] Seed::IKRandomSeedGenerator InitializeSeedGenerator(
	const std::optional< IterationState >& state, const VecXd& initial_joints ) const;

void PerformIteration(
	const IKProblem& problem,
	IterationState& state,
	SolverBuffers& buffers ) const;

void WrapJoints( VecXd& joints ) const;

bool UpdateBuffer(
	const Mat4d& target,
	const VecXd& joints,
	SolverBuffers& buffers ) const;

void UpdateHistory(
	int iteration,
	const IterationState& state,
	SolverHistory& history ) const;

void LineSearch(
	const Mat4d& target,
	IterationState& state,
	SolverBuffers& buffers ) const;

[[nodiscard]] DLSSolverState EvaluateConvergence(
	const IKProblem& problem,
	const IterationState& state,
	int iteration ) const noexcept;

void UpdateDeltaQPrimary(
	Eigen::LDLT< MatXd >& ldlt_solver,
	const MatXd& damped,
	const VecXd& gradient,
	VecXd& dq_primary ) const;

void UpdateDeltaQSecondary(
	const VecXd& joints,
	const MatXd& jacobian,
	const MatXd& jacobian_psi,
	VecXd& dq_secondary ) const;

void UpdateDeltaQ(
	Eigen::LDLT< MatXd >& ldlt_solver,
	const MatXd& damped,
	const VecXd& gradient,
	const VecXd& joints,
	const MatXd& jacobian,
	const MatXd& jacobian_psi,
	VecXd& dq ) const;

void ClampDeltaQ( VecXd& dq ) const;

void UpdateErrorConvergence(
	const IKProblem& problem,
	double last_error,
	double current_error,
	IterationState& state ) const;

[[nodiscard]] Model::RandomType ChooseSeedRandomType(
	int iteration,
	const IterationState& state,
	const SolverHistory& history ) const;

void UpdateSeedGenerator(
	int iteration,
	const IterationState& state,
	const SolverHistory& history,
	Seed::IKRandomSeedGenerator& seed_generator ) const;

void PrintIteration( const IterationState& state, const SolverBuffers& buffers ) const;
void PrintState( const IterationState& state ) const;
void PrintHistory( const SolverHistory& history ) const;
void PrintBuffer( const SolverBuffers& buffers ) const;

[[nodiscard]] double UnreachableError( double error, double error_reachable ) const noexcept {
	return std::max( 0.0, sqrt( error * error - error_reachable * error_reachable ) );
}

[[nodiscard]] constexpr double SmallError() const noexcept {
	return parameters_.rotation_weight * small_rotation_error +
	       parameters_.translation_weight * small_translation_error;
}

[[nodiscard]] constexpr double MediumError() const noexcept {
	return parameters_.rotation_weight * medium_rotation_error +
	       parameters_.translation_weight * medium_translation_error;
}

[[nodiscard]] constexpr double LargeError() const noexcept {
	return parameters_.rotation_weight * large_rotation_error +
	       parameters_.translation_weight * large_translation_error;
}
};
}
