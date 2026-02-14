#pragma once

#include "Global.hpp"

#include <memory>

namespace SOArm100::Kinematics
{
struct BaseJointModel
{
	Vec3d reference_direction{ Vec3d::Zero() };
};

using BaseJointModelUniqueConstPtr = std::unique_ptr< const BaseJointModel >;
}