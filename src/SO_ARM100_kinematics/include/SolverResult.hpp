#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics
{
enum class SolverState
{
	None,
	Success,
	Unreachable,
	Singularity,
};

struct SolverResult
{
	SolverState state;
	VecXd joints;

	SolverResult( size_t n ) :
		state( SolverState::None ),
		joints( n )
	{
	}

	SolverResult( SolverState result_state, VecXd result_joints ) :
		state( result_state ),
		joints( result_joints )
	{
	}

	inline bool Unreachable() const {
		return state == SolverState::Unreachable;
	}

	inline bool Success() const {
		return state == SolverState::Success;
	}

	inline bool Singularity() const {
		return state == SolverState::Singularity;
	}
};

static SolverState GetSolverState(
	const std::vector< SolverResult >& results )
{
	SolverState state = SolverState::Success;

	for ( auto result : results )
	{
		if ( result.Unreachable() )
			return SolverState::Unreachable;
		if ( result.Singularity() )
			state = SolverState::Singularity;
	}

	return state;
}
}