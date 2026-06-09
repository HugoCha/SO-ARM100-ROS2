#include "ModelAnalyzer/FallbackFabrikAnalyzer.hpp"

#include "Global.hpp"

#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Utils/KinematicsUtils.hpp"

#include <optional>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

std::optional< FallbackFabrikJointGroup > FallbackFabrikAnalyzer::Analyze(
	const JointChain& joint_chain,
	const Mat4d& home,
	const std::optional< JointGroup >& base_group,
	const std::optional< PlanarNRJointGroup >& planar_group,
	const std::optional< WristJointGroup >& wrist_group  )
{
	if ( joint_chain.Empty() )
		return std::nullopt;

    const int n_joints = joint_chain.GetActiveJointCount(); 

    int base_cnt = base_group ? base_group->Size() : 0;
    int planar_cnt = planar_group ? planar_group->Size() : 0;
    int wrist_cnt = wrist_group ? wrist_group->Size() : 0;

    if ( base_cnt + planar_cnt + wrist_cnt == n_joints )
        return std::nullopt;

    if ( wrist_group )
    {
        return FallbackFabrikJointGroup( 
            0, 
            wrist_group->FirstIndex(), 
            wrist_group->GetWristCenter() );
    }

	return FallbackFabrikJointGroup(
        0,
        n_joints,
        home
    );
}

// ------------------------------------------------------------

}