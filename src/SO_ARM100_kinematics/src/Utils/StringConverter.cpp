#include "Utils/StringConverter.hpp"

#include "Global.hpp"

#include "Heuristic/IKHeuristicState.hpp"
#include "Heuristic/IKPresolution.hpp"
#include "Model/Geometry/Pose.hpp"
#include "Model/Joint/Joint.hpp"
#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/Joint/JointState.hpp"
#include "Model/Joint/JointType.hpp"
#include "Model/KinematicModel.hpp"
#include "Model/KinematicTopology/KinematicTopology.hpp"
#include "Model/Joint/Limits.hpp"
#include "Model/Joint/Link.hpp"
#include "Model/Skeleton/Articulation.hpp"
#include "Model/Skeleton/ArticulationState.hpp"
#include "Model/Skeleton/ArticulationType.hpp"
#include "Model/Skeleton/Bone.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "Model/Skeleton/Skeleton.hpp"
#include "Model/Skeleton/SkeletonState.hpp"
#include "DLS/DLSSolverParameters.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKSolution.hpp"
#include "Solver/IKSolverState.hpp"

#include <string>

namespace  SOArm100::Kinematics
{

// ============================================================
// Model
// ============================================================

namespace Model
{

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const Articulation& obj )
{
	os << "Articulation "
	   << obj.GetType()
	   << " Center: " << obj.Center().transpose()
	   << " Axis: " << obj.Axis().transpose();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const ArticulationState& obj )
{
	os << "Articulation state"
	   << " Center: " << obj.WorldTransform().translation().transpose()
	   << " Values: " << obj.GetJointValues().transpose();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const ArticulationType& obj )
{
	switch ( obj )
	{
	case ArticulationType::Revolute:
	{ os << "REVOLUTE"; return os; }
	case ArticulationType::Prismatic:
	{ os << "PRISMATIC"; return os; }
	case ArticulationType::Universal:
	{ os << "UNIVERSAL"; return os; }
	case ArticulationType::Spherical:
	{ os << "SPHERICAL"; return os; }
	}
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const Bone& obj )
{
	os << "Bone"
	   << " Origin: " << obj.Origin().transpose()
	   << " Direction: " << obj.Direction().transpose()
	   << " Length: " << obj.Length();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const BoneState& obj )
{
	os << "Bone State"
	   << " Origin: " << obj.Origin().transpose()
	   << " Direction: " << obj.Direction().transpose();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const Joint& obj )
{
	os << "Joint"
	   << " " << obj.GetName()
	   << " " << obj.GetType()
	   << " " << obj.GetTwist()
	   << " " << obj.GetLimits();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const JointChain& obj )
{
	os << "Joint Chain:" << std::endl;
	for ( int i = 0; i < obj.GetJointCount(); i++ )
	{
		os << std::to_string( i + 1 ) << ": " << *obj.GetJoints()[i];
		if ( i < obj.GetJointCount() - 1 )
			os << std::endl;
	}
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const JointGroup& obj )
{
	os << "Joint Group " << obj.name << std::endl;
	os << "Indices: ";
	bool first = true;
	for ( const auto& index : obj.indices )
	{
		if ( !first )
			os << ", ";
		first = false;
		os << index;
	}
	os << "\nGroup Home:\n" << obj.tip_home;
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const JointState& obj )
{
	os << "Joint State"
	   << " Origin: " << obj.Origin().transpose()
	   << " Axis: " << obj.Axis().transpose()
	   << " Value: " << obj.Value();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const JointType& obj )
{
	switch ( obj )
	{
	case JointType::FIXED:
	{ os << "FIXED"; return os; }
	case JointType::REVOLUTE:
	{ os << "REVOLUTE"; return os; }
	case JointType::PRISMATIC:
	{ os << "PRISMATIC"; return os; }
	}
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const KinematicModel& obj )
{
	os << "Kinematic Model:" << std::endl;
	os << *obj.GetChain() << std::endl;
	os << "Home" << std::endl << obj.GetHomeConfiguration() << std::endl;
	os << *obj.GetSkeleton() << std::endl;
	os << obj.GetTopology();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const KinematicTopology& obj )
{
	os << "Kinematic Topologies: ";
	bool has_topology = false;
	for ( const std::string& topology : KinematicTopologies )
	{
		if ( obj.Get( topology ) )
		{
			os << std::endl;
			os << *obj.Get( topology );
			has_topology = true;
		}
	}

	if ( !has_topology )
		os << "None";

	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const Limits& obj )
{
	os << "Limits"
	   << " Min: " << obj.Min()
	   << " Max: " << obj.Max();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const Link& obj )
{
	os << "Link " << obj.GetName()
	   << " Global tf: \n" << obj.HomeTransform()
	   << " Joint tf:  \n" << obj.ParentJointTransform()
	   << " Length:    " << obj.Length();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const Pose& obj )
{
	os << "Pose"
	   << " Origin: " << obj.origin.transpose()
	   << " Axis: " << obj.axis.transpose();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const Skeleton& obj )
{
	os << "Skeleton:" << std::endl;
	for ( int i = 0; i < obj.ArticulationCount(); i++ )
	{
		os << std::to_string( i + 1 ) << ": " << *obj.Articulation( i ) << std::endl;
		if ( i < obj.BonesCount() )
			os << std::to_string( i + 1 ) << ": " << *obj.Bone( i ) << std::endl;
	}
	os << "Total Length: " << obj.TotalLength();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SkeletonState& obj )
{
	os << "Skeleton State" << std::endl;
	auto bone_states = obj.GetBoneStates();
	auto articulation_states = obj.GetArticulationStates();

	for ( int i = 0; i < articulation_states.size(); i++ )
	{
		os << std::to_string( i + 1 ) << ": " << *articulation_states[i] << std::endl;
		if ( i < bone_states.size() )
			os << std::to_string( i + 1 ) << ": " << bone_states[i] << std::endl;
	}

	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const Twist& obj )
{
	os << "Twist"
	   << " w: " << obj.Omega().transpose()
	   << " v: " << obj.V().transpose();
	return os;
}

}

// ============================================================
// Solver
// ============================================================

namespace Solver
{

std::ostream& operator << ( std::ostream& os, const IKProblem& obj )
{
	os << "IKProblem" << std::endl;
	os << "tol: " << obj.tolerance
	   << " timeout(ms): " << obj.timeout_ms << std::endl;
	os << "Seed: " << obj.seed.transpose() << std::endl;
	os << "Target: " << std::endl << obj.target;
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const IKSolverState& obj )
{
	switch ( obj )
	{
	case IKSolverState::BestPossible: os << "Best Possible"; break;
	case IKSolverState::Converged: os << "Converged"; break;
	case IKSolverState::MaxIterations: os << "Max Iterations"; break;
	case IKSolverState::NotRun: os << "Not Run"; break;
	case IKSolverState::Unreachable: os << "Unreachable"; break;
	}
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const IKSolution& obj )
{
	os << "IKSolution" << std::endl;
	os << "State: " << obj.state << std::endl;
	os << "Error: " << obj.error << std::endl;
	os << "Iter : " << obj.iterations << std::endl;
	os << "Score: " << obj.score << std::endl;
	os << "Joint: " << obj.joints.transpose();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const DLSSolverParameters& obj )
{
	os << "DLS Solver Parameters" << std::endl;
	os << "Max iter        = " << obj.max_iterations << std::endl;
	os << "Max stalle iter = " << obj.max_stalle_iterations << std::endl;
	os << "Gradient tol    = " << obj.gradient_tolerance << std::endl;
	os << "Min damping     = " << obj.min_damping << std::endl;
	os << "Max damping     = " << obj.max_damping << std::endl;
	os << "Min step        = " << obj.min_step << std::endl;
	os << "Max step        = " << obj.max_step << std::endl;
	os << "Line search     = " << obj.line_search_factor << std::endl;
	os << "Min SV tol      = " << obj.min_sv_tolerance << std::endl;
	os << "Max dq          = " << obj.max_dq << std::endl;
	os << "Translation w   = " << obj.translation_weight << std::endl;
	os << "Rotation w      = " << obj.rotation_weight << std::endl;
	return os;
}

// ------------------------------------------------------------

}

// ============================================================
// Heurisitic
// ============================================================

namespace Heuristic
{

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const IKPresolution& obj )
{
	os << "IKPresolution" << std::endl;
	os << "State: " << obj.state << std::endl;
	os << "Error: " << obj.error << std::endl;
	os << "Iter : " << obj.iterations << std::endl;
	os << "Joint: " << obj.joints.transpose() << std::endl;
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const IKHeuristicState& obj )
{
	switch ( obj )
	{
	case IKHeuristicState::Success: os << "Sucess"; break;
	case IKHeuristicState::PartialSuccess: os << "Partial Success"; break;
	case IKHeuristicState::Fail: os << "Fail"; break;
	}
	return os;
}

// ------------------------------------------------------------

}

}