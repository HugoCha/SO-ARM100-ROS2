#include "ModelAnalyzer/TopologyAnalyzer.hpp"

#include "Global.hpp"
#include "Model/Joint/JointChain.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/KinematicTopology/KinematicTopology.hpp"
#include "ModelAnalyzer/BaseAnalyzer.hpp"
#include "ModelAnalyzer/FallbackFabrikAnalyzer.hpp"
#include "ModelAnalyzer/PlanarNRAnalyzer.hpp"
#include "ModelAnalyzer/WristAnalyzer.hpp"
#include <optional>

namespace SOArm100::Kinematics::Model
{

// ------------------------------------------------------------

KinematicTopology TopologyAnalyzer::Analyze( const Model::JointChain& chain, const Mat4d& home )
{
	KinematicTopology topology;

	auto wrist_group = WristAnalyzer::Analyze( chain, home );

	int planar_analyze_start = 0;
	int planar_analyze_count = wrist_group ?
	                           wrist_group->FirstIndex():
	                           chain.GetActiveJointCount();
	Mat4d planar_home = wrist_group ? 
		wrist_group->GetWristCenter()  :
		home;

	auto planar_group = PlanarNRAnalyzer::Analyze( chain, planar_home, planar_analyze_start, planar_analyze_count );
	if ( !planar_group.has_value() )
	{
		planar_analyze_start = 1;
		planar_analyze_count -= 1;
		planar_group = PlanarNRAnalyzer::Analyze( chain, planar_home, planar_analyze_start, planar_analyze_count );
	}

	auto base_group = BaseAnalyzer::Analyze( chain, home, planar_group, wrist_group );

	auto fallback_group = FallbackFabrikAnalyzer::Analyze( chain, home, base_group, planar_group, wrist_group );

	if ( base_group )
		topology.Add( *base_group );

	if ( wrist_group )
		topology.Add( *wrist_group );

	if ( planar_group )
		topology.Add( *planar_group );

	if ( fallback_group )
		topology.Add( *fallback_group );

	return topology;
}

// ------------------------------------------------------------

}