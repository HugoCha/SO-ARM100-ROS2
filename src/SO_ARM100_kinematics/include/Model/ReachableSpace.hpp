#pragma once

#include "Global.hpp"

#include <memory>

namespace SOArm100::Kinematics::Model
{
class ReachableSpace
{
public:
virtual ~ReachableSpace() = default;

virtual bool IsUnreachable( const Mat4d& target ) const = 0;
};

using ReachableSpaceUniqueConstPtr = std::unique_ptr< const ReachableSpace >; 
}