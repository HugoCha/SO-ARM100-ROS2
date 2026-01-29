#pragma once

#include "KinematicsSolver.hpp"
#include "Types.hpp"

namespace SOArm100::Kinematics
{
class DLSKinematicsSolver : public KinematicsSolver
{
public:
DLSKinematicsSolver();
~DLSKinematicsSolver();

virtual bool InverseKinematic(
	const geometry_msgs::msg::Pose& target_pose,
	std::vector< double >& joint_angles ) override;

private:
double min_damping_factor_;
double max_damping_factor_;
double min_sv_factor_;
double min_step_;
double epsilon_step_;
int max_iterations_;
double error_tolerance_;

VecXd InitializeJointAngles();
VecXd RandomValidJointAngles();
bool Initialize(
	const Mat4d& target,
	const VecXd& initial_joints,
	Mat4d& fk,
	MatXd& jacobian,
	VecXd& pose_error,
	double& error,
	double& damping_factor,
	double& step,
	MatXd& damped,
	VecXd& dq );

bool IsSolverStale( int fk_iteration, double step, double damping_factor );

double MinSingularValue( const MatXd& jacobian );
double AdaptativeStep( double min_singular_value );
double AdaptativeDampingFactor( double min_singular_value );
double BacktrackStep( double step );
double BacktrackDampingFactor( double damping );
};
}
