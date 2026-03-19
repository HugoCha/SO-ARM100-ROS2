#pragma once

#include <optional>

namespace SOArm100::Kinematics::Model
{
class KinematicModel;
class JointGroup;

class WristAnalyzer
{
public:
static std::optional< JointGroup > Analyze(
	const KinematicModel& model );
};
}