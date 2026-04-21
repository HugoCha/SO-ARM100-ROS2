#include "Model/Skeleton/PrismaticArticulationBoneConstraint.hpp"
#include "Model/Skeleton/RevoluteArticulationBoneConstraint.hpp"
#include "Model/Skeleton/UniversalArticulationBoneConstraint.hpp"

#include "Global.hpp"

#include "KinematicTestBase.hpp"
#include "Model/Joint/Joint.hpp"
#include "Model/Skeleton/Articulation.hpp"
#include "Model/Skeleton/ArticulationType.hpp"
#include "Model/Skeleton/Bone.hpp"
#include "Model/Skeleton/BoneState.hpp"
#include "Utils/KinematicsUtils.hpp"
#include "Utils/StringConverter.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <ostream>

namespace SOArm100::Kinematics::Test
{

// ============================================================
// Prismatic Articulation Bone Constraint Test
// ============================================================

class PrismaticArticulationBoneConstraintTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	prismatic_joint_ = MakePrismaticJoint( Vec3d( 0, 0, 1 ), Vec3d( 0, 0, 0 ), 0.0, 1.0 );
	std::vector< Model::JointConstPtr > prismatic_joints = { prismatic_joint_ };
	prismatic_articulation_ = std::make_shared< const Model::Articulation >(
		Model::ArticulationType::Prismatic,
		prismatic_joints,
		Vec3d( 0, 0, 0 ) );
	bone_off_axis_ = std::make_shared< const Model::Bone >( Vec3d( 0, 0, 0 ), Vec3d( 1, 0, 0 ) );
	bone_on_axis_ = std::make_shared< const Model::Bone >( Vec3d( 0, 0, 0 ), Vec3d( 0, 0, 0.1 ) );
}

void TearDown() override {
}

Model::JointConstPtr prismatic_joint_;
Model::ArticulationConstPtr prismatic_articulation_;
Model::BoneConstPtr bone_on_axis_;
Model::BoneConstPtr bone_off_axis_;
};

// ------------------------------------------------------------

TEST_F( PrismaticArticulationBoneConstraintTest, ApplyConstraints_BoneOnAxisWithinLimits_ExpectedResult )
{
	auto constraint = Model::PrismaticArticulationBoneConstraint(
		prismatic_articulation_,
		bone_on_axis_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );
	bone_state.Origin() = Vec3d::Zero();
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	Vec3d expected_bone_origin = Vec3d::Zero();
	Vec3d expected_bone_direction = Vec3d( 0, 0, 0.5 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationBoneConstraintTest, ApplyConstraints_BoneOnAxisAboveUpperLimit_ExpectedResult )
{
	auto constraint = Model::PrismaticArticulationBoneConstraint(
		prismatic_articulation_,
		bone_on_axis_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );
	bone_state.Origin() = Vec3d::Zero();
	bone_state.Direction() = Vec3d( 0.5, 0.5, 1.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	Vec3d expected_bone_origin = Vec3d::Zero();
	Vec3d expected_bone_direction = Vec3d( 0, 0, 1.1 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationBoneConstraintTest, ApplyConstraints_BoneOnAxisBelowLowerLimit_ExpectedResult )
{
	auto constraint = Model::PrismaticArticulationBoneConstraint(
		prismatic_articulation_,
		bone_on_axis_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );
	bone_state.Origin() = Vec3d::Zero();
	bone_state.Direction() = Vec3d( 0.5, 0.5, -1.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	Vec3d expected_bone_origin = Vec3d::Zero();
	Vec3d expected_bone_direction = Vec3d( 0, 0, 0.1 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationBoneConstraintTest, ApplyConstraints_BoneOnAxisWithRotationTranslation_ExpectedResult )
{
	auto constraint = Model::PrismaticArticulationBoneConstraint(
		prismatic_articulation_,
		bone_on_axis_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );
	bone_state.Origin() = Vec3d( 1.1, 1, 1 );
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitY() );
	auto center = Vec3d::Ones();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	Vec3d expected_bone_origin = Vec3d( 1, 1, 1 );
	Vec3d expected_bone_direction = Vec3d( 0, 0.5, 0 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationBoneConstraintTest, ApplyConstraints_BoneOffAxisWithinLimits_ExpectedResult )
{
	auto constraint = Model::PrismaticArticulationBoneConstraint(
		prismatic_articulation_,
		bone_off_axis_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d::Zero();
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	Vec3d expected_bone_origin = Vec3d::Zero();
	Vec3d expected_bone_direction = Vec3d( 1, 0, 0.5 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationBoneConstraintTest, ApplyConstraints_BoneOffAxisAboveUpperLimit_ExpectedResult )
{
	auto constraint = Model::PrismaticArticulationBoneConstraint(
		prismatic_articulation_,
		bone_off_axis_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d::Zero();
	bone_state.Direction() = Vec3d( 0.5, 0.5, 1.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	Vec3d expected_bone_origin = Vec3d::Zero();
	Vec3d expected_bone_direction = Vec3d( 1, 0, 1 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationBoneConstraintTest, ApplyConstraints_BoneOffAxisBelowLowerLimit_ExpectedResult )
{
	auto constraint = Model::PrismaticArticulationBoneConstraint(
		prismatic_articulation_,
		bone_off_axis_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d::Zero();
	bone_state.Direction() = Vec3d( 0.5, 0.5, -1.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	Vec3d expected_bone_origin = Vec3d::Zero();
	Vec3d expected_bone_direction = Vec3d( 1, 0, 0 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( PrismaticArticulationBoneConstraintTest, ApplyConstraints_BoneOffAxisWithRotationTranslation_ExpectedResult )
{
	auto constraint = Model::PrismaticArticulationBoneConstraint(
		prismatic_articulation_,
		bone_off_axis_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d( 1.1, 1, 1 );
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitY() );
	auto center = Vec3d::Ones();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	Vec3d expected_bone_origin = Vec3d( 1, 1, 1 );
	Vec3d expected_bone_direction = Vec3d( 1, 0.5, 0 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ============================================================
// Revolute Articulation Bone Constraint Test
// ============================================================

class RevoluteArticulationBoneConstraintTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	// Revolute articulation (single joint)
	revolute_joint_ = MakeRevoluteJoint(
		Vec3d( 0, 0, 1 ),
		Vec3d( 1, 0, 0 ),
		-M_PI / 2,
		M_PI / 2 );
	std::vector< Model::JointConstPtr > revolute_joints = { revolute_joint_ };
	revolute_articulation_ = std::make_shared< const Model::Articulation >(
		Model::ArticulationType::Revolute,
		revolute_joints,
		Vec3d( 1, 0, 0 ) );

	bone_off_axis_ = std::make_shared< const Model::Bone >( Vec3d( 1, 0, 0 ), Vec3d( 1, 0, 0 ) );
	bone_on_axis_ = std::make_shared< const Model::Bone >( Vec3d( 1, 0, 0 ), Vec3d( 0, 0, 0.1 ) );
}

void TearDown() override {
}

Model::JointConstPtr revolute_joint_;
Model::ArticulationConstPtr revolute_articulation_;
Model::BoneConstPtr bone_on_axis_;
Model::BoneConstPtr bone_off_axis_;
};

// ------------------------------------------------------------

TEST_F( RevoluteArticulationBoneConstraintTest, ApplyConstraints_BoneOnAxisWithinLimits_ExpectedResult )
{
	auto constraint = Model::RevoluteArticulationBoneConstraint(
		revolute_articulation_,
		bone_on_axis_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );
	bone_state.Origin() = Vec3d( 1, 0, 0 );
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	Vec3d expected_bone_origin = Vec3d( 1, 0, 0 );
	Vec3d expected_bone_direction = Vec3d::Zero();

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( RevoluteArticulationBoneConstraintTest, ApplyConstraints_BoneOffAxisWithinLimits_ExpectedResult )
{
	auto constraint = Model::RevoluteArticulationBoneConstraint(
		revolute_articulation_,
		bone_off_axis_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d( 1, 0, 0 );
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	Vec3d expected_bone_origin = Vec3d( 1, 0, 0 );
	Vec3d expected_bone_direction =
		bone_off_axis_->Length() * Vec3d( 1, 1, 0 ).normalized();

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( RevoluteArticulationBoneConstraintTest, ApplyConstraints_BoneOffAxisAboveUpperLimit_ExpectedResult )
{
	auto constraint = Model::RevoluteArticulationBoneConstraint(
		revolute_articulation_,
		bone_off_axis_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d( 1, 0, 0 );
	bone_state.Direction() = Vec3d( -0.5, 0.5, 0.5 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	Vec3d expected_bone_origin = Vec3d( 1, 0, 0 );
	Vec3d expected_bone_direction =
		bone_off_axis_->Length() * Vec3d( 0, 1, 0 ).normalized();

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( RevoluteArticulationBoneConstraintTest, ApplyConstraints_BoneOffAxisBelowLowerLimit_ExpectedResult )
{
	auto constraint = Model::RevoluteArticulationBoneConstraint(
		revolute_articulation_,
		bone_off_axis_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d::Zero();
	bone_state.Direction() = Vec3d( -0.5, -0.5, 0.5 ).normalized();

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	Vec3d expected_bone_origin = Vec3d::Zero();
	Vec3d expected_bone_direction =
		bone_off_axis_->Length() * Vec3d( 0, -1, 0 );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( RevoluteArticulationBoneConstraintTest, ApplyConstraints_BoneOffAxisWithRotationTranslation_ExpectedResult )
{
	auto constraint = Model::RevoluteArticulationBoneConstraint(
		revolute_articulation_,
		bone_off_axis_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	bone_state.Origin() = Vec3d( 0, 0, 0 );
	bone_state.Direction() = Vec3d( 0.5, 0.5, 0.5 );

	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitY() );
	auto center = Vec3d( -1, 0, 0 );
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	Vec3d expected_bone_origin = Vec3d( 0, 0, 0 );
	Vec3d expected_bone_direction = bone_off_axis_->Length() * Vec3d( 1, 0, 1 ).normalized();

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ============================================================
// Universal Articulation Bone Constraint Test
// ============================================================

class UniversalArticulationBoneConstraintTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	// Universal articulation (two joints)
	auto joint1 = MakeRevoluteJoint(
		Vec3d( 1, 0, 0 ),
		Vec3d( 0, 0, 0 ),
		-M_PI / 2,
		M_PI / 2 );
	auto joint2 = MakeRevoluteJoint(
		Vec3d( 0, 1, 0 ),
		Vec3d( 0.1, 0.1, 0 ),
		-M_PI / 2,
		M_PI / 2 );
	std::vector< Model::JointConstPtr > universal_joints = { joint1, joint2 };
	universal_articulation_ = std::make_shared< const Model::Articulation >(
		Model::ArticulationType::Universal,
		universal_joints,
		Vec3d( 0.1, 0, 0 ) );

	bone_off_axis_ = std::make_shared< const Model::Bone >( Vec3d( 0.1, 0, 0 ), Vec3d( 0, 0.1, 1 ) );
	bone_on_axis_ = std::make_shared< const Model::Bone >( Vec3d( 0.1, 0, 0 ), Vec3d( 0, 1, 0 ) );
}

void TearDown() override {
}

Vec3d BoneOffAxisInternalDirection( double angle1, double angle2 )
{
	Iso3d T03 = Iso3d::Identity();
	T03.translate( Vec3d( 0.1, 0, 0 ) );
	T03.rotate( AngleAxis( angle1, Vec3d::UnitX() ) );
	T03.translate( Vec3d( 0, 0.1, 0 ) );
	T03.rotate( AngleAxis( angle2, Vec3d::UnitY() ) );
	T03.translate( Vec3d( 0, 0, 1 ) );

	return T03.translation() - bone_off_axis_->Origin();
}

Model::ArticulationConstPtr universal_articulation_;
Model::BoneConstPtr bone_on_axis_;
Model::BoneConstPtr bone_off_axis_;
};

// ------------------------------------------------------------

TEST_F( UniversalArticulationBoneConstraintTest, ApplyConstraints_BoneOffAxisFirstJointRotationWithinLimits_ExpectedResult )
{
	auto constraint = Model::UniversalArticulationBoneConstraint(
		universal_articulation_,
		bone_off_axis_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	Vec3d expected_bone_origin = Vec3d( 0.1, 0, 0 );
	Vec3d expected_bone_direction = BoneOffAxisInternalDirection( M_PI / 2, 0 );

	bone_state.Origin() = expected_bone_origin;
	bone_state.Direction() = expected_bone_direction;

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationBoneConstraintTest, ApplyConstraints_BoneOffAxisSecondJointRotationWithinLimits_ExpectedResult )
{
	auto constraint = Model::UniversalArticulationBoneConstraint(
		universal_articulation_,
		bone_off_axis_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	Vec3d expected_bone_origin = Vec3d( 0.1, 0, 0 );
	Vec3d expected_bone_direction = BoneOffAxisInternalDirection( 0, M_PI / 4 );

	bone_state.Origin() = expected_bone_origin;
	bone_state.Direction() = expected_bone_direction;

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationBoneConstraintTest, ApplyConstraints_BoneOffAxisCombinedRotationWithinLimits_ExpectedResult )
{
	auto constraint = Model::UniversalArticulationBoneConstraint(
		universal_articulation_,
		bone_off_axis_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	Vec3d expected_bone_origin = Vec3d( 0.1, 0, 0 );
	Vec3d expected_bone_direction = BoneOffAxisInternalDirection( M_PI / 4, M_PI / 4 );

	bone_state.Origin() = expected_bone_origin;
	bone_state.Direction() = expected_bone_direction;

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationBoneConstraintTest, ApplyConstraints_BoneOffAxisWithRotationTranslation_ExpectedResult )
{
	auto constraint = Model::UniversalArticulationBoneConstraint(
		universal_articulation_,
		bone_off_axis_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );
	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitY() );
	auto center = Vec3d( 1, 1, 1 );

	Vec3d expected_bone_origin = center;
	Vec3d expected_bone_direction = rotation * BoneOffAxisInternalDirection( M_PI / 6, -M_PI / 4 );

	bone_state.Origin() = expected_bone_origin;
	bone_state.Direction() = expected_bone_direction;

	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );


	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationBoneConstraintTest, ApplyConstraints_BoneOffAxisFirstJointRotationOutsideLimits_ExpectedResult )
{
	auto constraint = Model::UniversalArticulationBoneConstraint(
		universal_articulation_,
		bone_off_axis_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	Vec3d expected_bone_origin = Vec3d( 0.1, 0, 0 );
	Vec3d expected_bone_direction = BoneOffAxisInternalDirection( M_PI / 2, 0 );

	bone_state.Origin() = Vec3d( 0.1, 0, 0 );
	bone_state.Direction() = BoneOffAxisInternalDirection( 3 * M_PI / 4, 0 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationBoneConstraintTest, ApplyConstraints_BoneOffAxisSecondJointRotationOutsideLimits_ExpectedResult )
{
	auto constraint = Model::UniversalArticulationBoneConstraint(
		universal_articulation_,
		bone_off_axis_ );

	auto bone_state = Model::BoneState( bone_off_axis_ );

	Vec3d expected_bone_origin = Vec3d( 0.1, 0, 0 );
	Vec3d expected_bone_direction = BoneOffAxisInternalDirection( 0, M_PI / 2 );

	bone_state.Origin() = Vec3d( 0.1, 0, 0 );
	bone_state.Direction() = BoneOffAxisInternalDirection( 0, 3 * M_PI / 4 );

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationBoneConstraintTest, ApplyConstraints_BoneOnAxisWithinLimits_ExpectedResult )
{
	auto constraint = Model::UniversalArticulationBoneConstraint(
		universal_articulation_,
		bone_on_axis_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );

	Vec3d expected_bone_origin = Vec3d( 0.1, 0, 0 );
	Vec3d expected_bone_direction = Vec3d( 0, 1, 1 ).normalized();

	bone_state.Origin() = expected_bone_origin;
	bone_state.Direction() = expected_bone_direction;

	auto rotation = Quaternion::Identity();
	auto center = Vec3d::Zero();
	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

TEST_F( UniversalArticulationBoneConstraintTest, ApplyConstraints_BoneOnAxisWithRotationTranslation_ExpectedResult )
{
	auto constraint = Model::UniversalArticulationBoneConstraint(
		universal_articulation_,
		bone_on_axis_ );

	auto bone_state = Model::BoneState( bone_on_axis_ );
	auto rotation = Quaternion::FromTwoVectors( Vec3d::UnitZ(), Vec3d::UnitX() );
	auto center = Vec3d( 1, 1, 1 );

	Vec3d expected_bone_origin = center;
	Vec3d expected_bone_direction = rotation * Vec3d( 0, 1, 1 ).normalized();

	bone_state.Origin() = expected_bone_origin;
	bone_state.Direction() = expected_bone_direction;

	constraint.ApplyConstraint(
		rotation,
		center,
		bone_state );

	EXPECT_TRUE( bone_state.Origin().isApprox( expected_bone_origin ) )
	    << "Expected Origin   = " << expected_bone_origin.transpose() << std::endl
	    << "Constraint Origin = " << bone_state.Origin().transpose() << std::endl;

	EXPECT_TRUE( bone_state.Direction().isApprox( expected_bone_direction ) )
	    << "Expected Direction   = " << expected_bone_direction.transpose() << std::endl
	    << "Constraint Direction = " << bone_state.Direction().transpose() << std::endl;
}

// ------------------------------------------------------------

} // namespace SOArm100::Kinematics::Test
