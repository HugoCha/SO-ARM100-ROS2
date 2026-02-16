#pragma once

#include "BaseJointModel.hpp"
#include "NumericJointsModel.hpp"
#include "WristModel.hpp"

#include <optional>
#include <sys/types.h>

namespace SOArm100::Kinematics
{
enum class HybridSolverFlags : u_int8_t
{
	Base    = 1 << 0,
	Wrist   = 1 << 1,
	Numeric = 1 << 2,
	BaseWrist = Base | Wrist,
	NumericWrist = Numeric | Wrist,
	BaseNumericWrist = Base | Numeric | Wrist,
};

constexpr HybridSolverFlags operator | ( HybridSolverFlags a, HybridSolverFlags b ){
	return static_cast< HybridSolverFlags >(
		static_cast< uint8_t >( a ) | static_cast< uint8_t >( b )
		);
}

inline HybridSolverFlags& operator |= ( HybridSolverFlags& a, HybridSolverFlags b ){
	a = a | b;
	return a;
}

inline bool IsFlagSet( HybridSolverFlags flags, HybridSolverFlags flagToCheck ){
	return ( static_cast< uint8_t >( flags ) & static_cast< uint8_t >( flagToCheck ) ) != 0;
}

struct HybridSolverConfiguration
{
	std::optional< BaseJointModel > base_joint_model{ std::nullopt };
	std::optional< NumericJointsModel > numeric_joints_model{ std::nullopt };
	std::optional< WristModel > wrist_model{ std::nullopt };
	HybridSolverFlags solver_flags;
};
}