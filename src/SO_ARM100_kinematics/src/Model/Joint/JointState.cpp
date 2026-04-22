#include "Model/Joint/JointState.hpp"

#include "Model/Geometry/Pose.hpp"
#include "Model/Joint/Joint.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

JointState::JointState( const JointConstPtr& joint ) :
	joint_( joint ),
	pose_( { joint->Origin(), joint->Axis() } ),
	value_( joint->GetLimits().Within( 0 ) ? 0 : joint->GetLimits().Center() )
{
}

// ------------------------------------------------------------

}