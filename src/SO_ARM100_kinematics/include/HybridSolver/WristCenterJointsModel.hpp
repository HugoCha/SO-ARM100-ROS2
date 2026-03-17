#pragma once

#include "Global.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
struct WristCenterJointsModel
{
	int start_index{ 0 };
	size_t count{ 0 };
	Mat4d home_configuration{ Mat4d::Identity() };
};

using WristCenterJointsModelUniqueConstPtr = std::unique_ptr< const WristCenterJointsModel >;
}