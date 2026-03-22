#pragma once

#include "Global.hpp"

#include <vector>

namespace SOArm100::Kinematics::Model
{
class JointChain;
class JointGroup;

class PlanarNRAnalyzer
{
public:
static std::vector< JointGroup > Analyze( 
    const JointChain& chain, 
    const Mat4d& home,
    int start_idx,
    int count );
static std::vector< JointGroup > Analyze( 
    const JointChain& chain, 
    const Mat4d& home );

static bool CheckConsistency(
    const JointChain& chain, 
    const JointGroup& planar_group );
};
}