#pragma once

#include "Global.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
struct NumericJointsModel
{
	int start_index{ 0 };
	int count{ 0 };
	Mat4d home_configuration{ Mat4d::Identity() };
};

using NumericJointsModelUniqueConstPtr = std::unique_ptr< const NumericJointsModel >;
}