#pragma once

#include "Global.hpp"
#include "Model/Joint/Joint.hpp"

#include <optional>
#include <vector>

namespace SOArm100::Kinematics::Solver
{
struct UniversalSolution;
}

namespace SOArm100::Kinematics::Model
{
class UniversalModel
{
public:
[[nodiscard]] static std::optional< UniversalModel > ComputeModel(
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2 );

[[nodiscard]] std::span< const Model::JointConstPtr > GetJoints() const {
	return joints_;
}

[[nodiscard]] const Model::JointConstPtr GetJoint( int i ) const {
	return joints_[i];
}

Mat3d Recompose( const Vec2d& q ) const;
std::vector< Solver::UniversalSolution > Decompose( const Mat3d& R_target ) const;
std::vector< Solver::UniversalSolution > Decompose( const Vec3d& b0, const Vec3d& b1 ) const;

private:
std::vector< Model::JointConstPtr > joints_;

UniversalModel(
	const Model::JointConstPtr& joint1,
	const Model::JointConstPtr& joint2 );

Solver::UniversalSolution ComputeSolution(
    double theta0_sol,
    const Limits& l0,
    const Limits& l1,
    const Vec3d& a0,
    const Vec3d& a1,
    const Vec3d& b0,
    const Vec3d& b1 ) const;
};
}