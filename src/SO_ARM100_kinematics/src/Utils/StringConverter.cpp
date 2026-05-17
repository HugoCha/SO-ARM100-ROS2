#include "Utils/StringConverter.hpp"

#include "Global.hpp"

#include "Model/Geometry/Pose.hpp"
#include "Model/Joint/Joint.hpp"
#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/JointState.hpp"
#include "Model/Joint/JointType.hpp"
#include "Model/Joint/Limits.hpp"
#include "Model/Joint/Link.hpp"
#include "Model/Skeleton/Articulation.hpp"
#include "Model/Skeleton/ArticulationState.hpp"
#include "Model/Skeleton/ArticulationType.hpp"
#include "Model/Skeleton/Bone.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "Model/Skeleton/Skeleton.hpp"
#include "Model/Skeleton/SkeletonState.hpp"
#include "Solver/IKProblem.hpp"
#include "Solver/IKSolution.hpp"
#include "Solver/IKSolverState.hpp"
#include <string>

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Articulation& obj )
{
	os << "Articulation "
	   << obj.GetType()
	   << " Center: " << obj.Center().transpose()
	   << " Axis: " << obj.Axis().transpose();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::ArticulationState& obj )
{
	os << "Articulation state"
	   << " Center: " << obj.GlobalTransform().translation().transpose()
	   << " Values: " << obj.GetJointValues().transpose();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::ArticulationType& obj )
{
	switch ( obj )
	{
	case SOArm100::Kinematics::Model::ArticulationType::Revolute:
	{ os << "REVOLUTE"; return os; }
	case SOArm100::Kinematics::Model::ArticulationType::Prismatic:
	{ os << "PRISMATIC"; return os; }
	case SOArm100::Kinematics::Model::ArticulationType::Universal:
	{ os << "UNIVERSAL"; return os; }
	case SOArm100::Kinematics::Model::ArticulationType::Spherical:
	{ os << "SPHERICAL"; return os; }
	}
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Bone& obj )
{
	os << "Bone"
	   << " Origin: " << obj.Origin().transpose()
	   << " Direction: " << obj.Direction().transpose()
	   << " Length: " << obj.Length();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::BoneState& obj )
{
	os << "Bone State"
	   << " Origin: " << obj.Origin().transpose()
	   << " Direction: " << obj.Direction().transpose();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Joint& obj )
{
	os << "Joint "
	   << obj.GetType()
	   << " " << obj.GetTwist()
	   << " " << obj.GetLink()
	   << " " << obj.GetLimits();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::JointChain& obj )
{
	os << "Joint Chain" << std::endl;
	for ( int i = 0; i < obj.GetJointCount(); i++ )
	{
		os << std::to_string( i + 1 ) << ": " << *obj.GetJoints()[i];
		if ( i < obj.GetJointCount() - 1 )
			os << std::endl;
	}
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::JointState& obj )
{
	os << "Joint State"
	   << " Origin: " << obj.Origin().transpose()
	   << " Axis: " << obj.Axis().transpose()
	   << " Value: " << obj.Value();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::JointType& obj )
{
	switch ( obj )
	{
	case SOArm100::Kinematics::Model::JointType::FIXED:
	{ os << "FIXED"; return os; }
	case SOArm100::Kinematics::Model::JointType::REVOLUTE:
	{ os << "REVOLUTE"; return os; }
	case SOArm100::Kinematics::Model::JointType::PRISMATIC:
	{ os << "PRISMATIC"; return os; }
	}
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Limits& obj )
{
	os << "Limits"
	   << " Min: " << obj.Min()
	   << " Max: " << obj.Max();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Link& obj )
{
	os << "Link"
	   << " Origin: " << obj.GetJointOrigin().transpose()
	   << " Length: " << obj.GetLength();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Pose& obj )
{
	os << "Pose"
	   << " Origin: " << obj.origin.transpose()
	   << " Axis: " << obj.axis.transpose();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Skeleton& obj )
{
	os << "Skeleton" << std::endl;
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

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::SkeletonState& obj )
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

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Model::Twist& obj )
{
	os << "Twist"
	   << " w: " << obj.Omega().transpose()
	   << " v: " << obj.V().transpose();
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Solver::IKProblem& obj )
{
	os << "IKProblem" << std::endl;
	os << "rotation tol: " << obj.rotation_tolerance 
	   << " translation tol: " << obj.position_tolerance
	   << " timeout(ms): " << obj.timeout << std::endl;
	os << "Seed: " << obj.seed.transpose() << std::endl;
	os << "Target: " << std::endl << obj.target;
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Solver::IKSolverState& obj )
{
	switch ( obj ) 
	{
		case SOArm100::Kinematics::Solver::IKSolverState::BestPossible: os << "Best Possible"; break;
		case SOArm100::Kinematics::Solver::IKSolverState::Converged: os << "Converged"; break;
		case SOArm100::Kinematics::Solver::IKSolverState::MaxIterations: os << "Max Iterations"; break;
		case SOArm100::Kinematics::Solver::IKSolverState::MaxRestart: os << "Max Restart"; break;
		case SOArm100::Kinematics::Solver::IKSolverState::NotRun: os << "Not Run"; break;
		case SOArm100::Kinematics::Solver::IKSolverState::Unreachable: os << "Unreachable"; break;
	}
	return os;
}

// ------------------------------------------------------------

std::ostream& operator << ( std::ostream& os, const SOArm100::Kinematics::Solver::IKSolution& obj )
{
	os << "IKSolution" << std::endl;
	os << "State: " << obj.state << std::endl;
	os << "Error: " << obj.error << std::endl;
	os << "Score: " << obj.score << std::endl;
	os << "Joint: " << obj.joints.transpose();
	return os;
}

// ------------------------------------------------------------