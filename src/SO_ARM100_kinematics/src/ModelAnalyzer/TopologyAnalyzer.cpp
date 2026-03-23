#pragma once

#include "ModelAnalyzer/TopologyAnalyzer.hpp"

#include "Model/JointChain.hpp"
#include "Model/JointGroup.hpp"
#include "Model/KinematicTopology.hpp"
#include "ModelAnalyzer/BaseAnalyzer.hpp"
#include "ModelAnalyzer/PlanarNRAnalyzer.hpp"
#include "ModelAnalyzer/WristAnalyzer.hpp"

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

KinematicTopology TopologyAnalyzer::Analyze( const Model::JointChain& chain, const Mat4d& home )
{
	KinematicTopology topology;

	auto wrist_group = WristAnalyzer::Analyze( chain, home );
	auto base_group = BaseAnalyzer::Analyze( chain, home, wrist_group );

	int planar_analyze_start = base_group  ? base_group->indices.size() : 0;
	int planar_analyze_count = wrist_group ?
	                           wrist_group->FirstIndex() - planar_analyze_start :
	                           chain.GetActiveJointCount() - planar_analyze_start;

	auto planar_group = PlanarNRAnalyzer::Analyze( chain, home, planar_analyze_start, planar_analyze_count );

	if ( base_group )
		topology.Add( *base_group );

	if ( wrist_group )
		topology.Add( *wrist_group );

	if ( planar_group.size() == 1 )
	{
		topology.Add( planar_group[0] );
	}

	return topology;
}

// ------------------------------------------------------------

}