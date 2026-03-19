#pragma once

#include "Global.hpp"

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
	const MatXd& jacobian,
	double damping,
	double min_damping,
	double max_damping,
	double min_sv_tolerance );
};
}