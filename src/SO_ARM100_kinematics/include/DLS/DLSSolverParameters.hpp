#pragma once 

#include "Global.hpp"

namespace SOArm100::Kinematics::Solver
{
struct DLSSolverParameters
{
    int max_iterations;
    int max_stalle_iterations;
    double gradient_tolerance;
    double min_sv_tolerance;
    double min_step;
    double max_step;
    double line_search_factor;
    double min_damping;
    double max_damping;
    double max_dq;
    double translation_weight;
    double rotation_weight;

    explicit DLSSolverParameters( 
        int max_iterations,
        int max_stalled_iterations,
        double gradient_tolerance,
        double min_sv_tolerance,
        double min_step,
        double max_step,
        double line_search_factor,
        double min_damping,
        double max_damping,
        double max_dq,
        double translation_w,
        double rotation_w ) :
        max_iterations( max_iterations ),
        max_stalle_iterations( max_stalled_iterations ),
        gradient_tolerance( gradient_tolerance ),
        min_sv_tolerance( min_sv_tolerance ),
        min_step( min_step ),
        max_step( max_step ),
        line_search_factor( line_search_factor ),
        min_damping( min_damping ),
        max_damping( max_damping ),
        max_dq( max_dq ),
        translation_weight( translation_w ),
        rotation_weight( rotation_w )
    {}

    [[nodiscard]] constexpr double RotationWeightSqrt() const noexcept {
        return sqrt( rotation_weight );
    }

    [[nodiscard]] constexpr double TranslationWeightSqrt() const noexcept {
        return sqrt( translation_weight );
    }

    [[nodiscard]] constexpr double AdaptativeDampingCoefficient() const noexcept {
        return -std::log( 0.99 ) / ( min_sv_tolerance * min_sv_tolerance );
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

struct DefaultDLSSolverParameters : DLSSolverParameters
{
    DefaultDLSSolverParameters() :
        DLSSolverParameters( 300, 2, 1e-8, 1e-4, 0.25, 1.0, 0.5, 5e-4, 0.1, 0.8, 9.0, 1.0 )
    {}
};

struct ExtraFastDLSSolverParameters : DLSSolverParameters
{
    ExtraFastDLSSolverParameters() :
        DLSSolverParameters( 50, 1, 1e-7, 1e-3, 1.0, 1.0, 0.5, 1e-4, 0.05, 1.5, 9.0, 1.0 )
    {}
};

struct FastDLSSolverParameters : DLSSolverParameters
{
    FastDLSSolverParameters() :
        DLSSolverParameters( 150, 1, 1e-7, 1e-3, 0.5, 1.0, 0.5, 1e-3, 0.1, 1.2, 9.0, 1.0 )
    {}
};

struct RobustDLSSolverParameters : DLSSolverParameters
{
    RobustDLSSolverParameters() :
        DLSSolverParameters( 800, 3, 1e-8, 1e-3, 0.1, 1.0, 0.5, 1e-4, 0.15, 0.8, 9.0, 1.0 )
    {}
};

}