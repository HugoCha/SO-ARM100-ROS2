#pragma once

#include "AdaptativeDamping.hpp"
#include "Global.hpp"

#include <memory>

namespace SOArm100::Kinematics
{

enum class SolverType;

// ------------------------------------------------------------

class SugiharaAdaptativeDampingFactory
{
public:
    static std::unique_ptr< AdaptativeDamping > Get( 
        SolverType type,
    	const MatXd& jacobian,
        const VecXd& error,
        double min_damping,
        double max_damping,
        double min_sv_tolerance );
};

// ------------------------------------------------------------

class SugiharaAdaptativeDamping : public AdaptativeDamping
{
public:
SugiharaAdaptativeDamping(
	const MatXd& jacobian,
	double min_damping,
	double max_damping,
	double min_sv_tolerance );

virtual double Damping() const override;

protected:
MatXd manipulability_;

const double min_damping_;
const double max_damping_;
const double min_sv_tolerance_;

double min_sv_;

double SugiharaDamping(
	double min_damping,
	double max_damping,
	double min_sv,
	double min_sv_tolerance ) const;
};

// ------------------------------------------------------------

class PositionSugiharaAdaptativeDamping : public SugiharaAdaptativeDamping
{
public:
PositionSugiharaAdaptativeDamping(
	const MatXd& jacobian,
	double min_damping,
	double max_damping,
	double min_sv_tolerance );
};

// ------------------------------------------------------------

class OrientationSugiharaAdaptativeDamping : public SugiharaAdaptativeDamping
{
public:
OrientationSugiharaAdaptativeDamping(
	const MatXd& jacobian,
    const VecXd& error,
	double min_damping,
	double max_damping,
	double min_sv_tolerance );

virtual double Damping() const override;

private:
    const VecXd& error_;
};

// ------------------------------------------------------------

}