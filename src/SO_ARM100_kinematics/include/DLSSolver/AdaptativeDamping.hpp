#pragma once

#include "Global.hpp"
#include "SolverType.hpp"

namespace SOArm100::Kinematics
{
class AdaptativeDamping
{
public:
static double LinearDamping(
	double damping,
	double min_damping,
	double max_damping,
	double alpha
	);

static double ManipulabilityDeterminant(
	const MatXd& jacobian,
	double min_damping,
	double max_damping,
	double epsilon
	);

static double ManipulabilityTrace(
	const MatXd& jacobian,
	double min_damping,
	double max_damping,
	double epsilon
	);

static double ManipulabilityMinSV(
	SolverType type,
	const MatXd& jacobian,
	const VecXd& error,
	double damping,
	double min_damping,
	double max_damping,
	double min_sv_tolerance );

private:
static double ManipulabilityMinSV(
	const MatXd& jacobian,
	double damping,
	double min_damping,
	double max_damping,
	double min_sv_tolerance );

static double ManipulabilityMinSVPosition(
	const MatXd& jacobian,
	double damping,
	double min_damping,
	double max_damping,
	double min_sv_tolerance );

static double ManipulabilityMinSVOrientation(
	const MatXd& jacobian,
	const VecXd& error,
	double damping,
	double min_damping,
	double max_damping,
	double min_sv_tolerance );
};
}