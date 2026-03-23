#pragma once

#include "Global.hpp"

namespace SOArm100::Kinematics::Model
{
class JointChain;
class KinematicTopology;

class TopologyAnalyzer
{
public:
static KinematicTopology Analyze(
	const Model::JointChain& chain,
	const Mat4d& home );
};
}