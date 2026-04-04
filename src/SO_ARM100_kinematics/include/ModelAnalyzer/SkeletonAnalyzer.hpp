#pragma once

#include "Global.hpp"

#include <memory>
#include <span>
#include <vector>

namespace SOArm100::Kinematics::Model
{
class Articulation;
class Bone;
class Joint;
class JointChain;
class JointGroup;
class Skeleton;

using ArticulationConstPtr = std::shared_ptr< const Articulation >;
using BoneConstPtr = std::shared_ptr< const Bone >;
using JointConstPtr = std::shared_ptr< const Joint >;
using SkeletonConstPtr = std::shared_ptr< const Skeleton >;

class SkeletonAnalyzer
{
public:
static SkeletonConstPtr Analyze(
	const std::span< const JointConstPtr >& joints,
	const Mat4d& tip );

static SkeletonConstPtr Analyze(
    const std::span< const JointConstPtr >& joints,
    const JointGroup& joint_group );

static std::vector< std::shared_ptr< const Articulation >> AnalyzeArticulations(
    const std::span< const JointConstPtr >& joints,
    const Mat4d& tip );

private:
static Skeleton Create(
	const std::span< const JointConstPtr >& joints,
	const Mat4d& tip );

static std::vector< JointConstPtr > ExtractGroupJoints(
	const std::span< const JointConstPtr >& joints,
	const Model::JointGroup& group );

static std::vector< BoneConstPtr > ComputeBones(
	const std::span< const ArticulationConstPtr >& articulations,
	const Mat4d& tip );

static double ComputeTotalLength(
	const std::span< const ArticulationConstPtr >& articulations,
	const std::span< const BoneConstPtr >& bones,
	const Mat4d& tip );

static std::vector< JointConstPtr > FilterJoints( 
    const std::span< const JointConstPtr >& joints );

static std::shared_ptr< Articulation > ExtractArticulationFromJoints(
	JointConstPtr joint1 );

static std::shared_ptr< Articulation > ExtractArticulationFromJoints(
	JointConstPtr joint1,
	JointConstPtr joint2 );

static std::shared_ptr< Articulation > ExtractArticulationFromJoints(
    JointConstPtr joint1,
    JointConstPtr joint2,
    JointConstPtr joint3 );

static std::shared_ptr< Articulation > ExtractArticulationFromLastJoints(
	const std::span< const JointConstPtr >& joints,
	const Mat4d& tip );
};
}