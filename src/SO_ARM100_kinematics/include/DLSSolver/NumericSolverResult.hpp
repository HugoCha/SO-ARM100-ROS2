#pragma once

#include "Global.hpp"

#include "NumericSolverState.hpp"

namespace SOArm100::Kinematics
{
struct NumericSolverResult
{
	NumericSolverState state;
	VecXd joints;
	double final_error;
	int iterations_used;

	[[nodiscard]] bool Success() const noexcept {
		return state == NumericSolverState::Converged;
	}

private:
	static constexpr const std::string SolverStateToString( const NumericSolverState& solver_state )
	{
		switch ( solver_state )
		{
		case NumericSolverState::Converged: return "Converged";
		case NumericSolverState::Improving: return "Improving";
		case NumericSolverState::Stalled: return "Stalled";
		case NumericSolverState::MaxIterations: return "MaxIterations";
		case NumericSolverState::Failed: return "Failed";
		default: return "";
		}
	}

	friend std::ostream& operator << ( std::ostream& os, const NumericSolverResult& obj ){
		os << "{ State :" << NumericSolverResult::SolverStateToString( obj.state ) << ", "
		   << "Error :" << obj.final_error << ", "
		   << "Iteration :" << obj.iterations_used << ", "
		   << "Joints :" << obj.joints.transpose() << " }";
		return os;
	}
};
}