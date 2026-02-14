#pragma once

#include "Global.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
enum class WristType
{
	None,
	Revolute1,
	Revolute2,
	Revolute3,
};

struct WristModel
{
	WristType type { WristType::None };
	int start_index{ 0 };
	int count{ 0 };
	Vec3d center_at_home{ Vec3d::Zero() };
	Mat4d tcp_in_wrist_at_home { Mat4d::Identity() };
	Mat4d tcp_in_wrist_at_home_inv { Mat4d::Identity() };
};

using WristModelUniqueConstPtr = std::unique_ptr< const WristModel >;

}