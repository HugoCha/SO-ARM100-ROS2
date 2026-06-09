#pragma once

#include "PipelineSolver/IKPipeline.hpp"
#include <memory>

namespace SOArm100::Kinematics::Model
{
class KinematicModel;
using KinematicModelConstPtr = std::shared_ptr< const KinematicModel >;
}

namespace SOArm100::Kinematics::Solver
{
class PipelineSolver;
struct PipelineSolverParameters;

class PipelineSolverInitializer
{
public:
static std::unique_ptr< const Solver::IKPipeline > InitializeSinglePipeline(
	Model::KinematicModelConstPtr model );

static std::unique_ptr< const Solver::PipelineSolver > InitializeMultiplePipeline(
	Model::KinematicModelConstPtr model,
	const PipelineSolverParameters& parameters );
};
}
