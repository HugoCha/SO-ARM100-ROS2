#include "ModelAnalyzer/TopologyAnalyzer.hpp"

#include "RobotModelTestData.hpp"

#include "KinematicTestBase.hpp"
#include "Model/Joint/JointGroup.hpp"
#include "Model/KinematicModel.hpp"
#include "Model/KinematicTopology/KinematicTopology.hpp"
#include "Utils/StringConverter.hpp"

#include <gtest/gtest.h>
#include <optional>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class TopologyAnalyzerTest : public KinematicTestBase
{
protected:
void SetUp() override
{
	model_name_ = "Universal Robot";
	model_ = Data::GetAllRobots()[model_name_];
}

void TearDown() override
{
}

protected:
std::string model_name_;

std::vector< std::string > KnownJointGroups(){
	return {
	    Model::revolute_base_name,
	    Model::planarNR_name,
	    Model::prismatic_base_name,
	    Model::wrist_name,
	};
}

bool CheckEquality( const Model::KinematicTopology& expected, const Model::KinematicTopology& result )
{
	for ( const auto& group_name : KnownJointGroups() )
	{
		if ( expected.Get( group_name ).has_value() != result.Get( group_name ).has_value() )
		{
			std::cout << "Mismatch for group " << group_name << std::endl;
			std::cout << "expect has value: " << expected.Get( group_name ).has_value() << std::endl;
			std::cout << "result has value: " << result.Get( group_name ).has_value() << std::endl;
			return false;
		}

		if ( expected.Get( group_name ).has_value() )
		{
			auto expected_group = *expected.Get( group_name );
			auto result_group = *result.Get( group_name );

			if ( !( expected_group.Size() == result_group.Size() &&
			        expected_group.indices == result_group.indices &&
			        expected_group.tip_home.isApprox( result_group.tip_home ) &&
			        expected_group.name == result_group.name ) )
			{

				std::cout << "Mismatch for " << group_name << std::endl;
				std::cout << "expect " << expected_group << std::endl;
				std::cout << "result " << result_group << std::endl;
				return false;
			}
		}
	}
	return true;
}

};

// ------------------------------------------------------------

TEST_F( TopologyAnalyzerTest, Analyze_ReturnExpected )
{
	const auto& chain = *model_->GetChain();
	const auto& home = model_->GetHomeConfiguration();
	auto analyze_topology = Model::TopologyAnalyzer::Analyze( chain, home );

	EXPECT_TRUE( CheckEquality( model_->GetTopology(),  analyze_topology ) )
	    << "Analyze fail for robot " << model_name_ << "Failed";
}

// ------------------------------------------------------------

TEST_F( TopologyAnalyzerTest, Analyze_ReturnExpected_AllRobots )
{
	for ( const auto& robot : Data::GetAllRobots() )
	{
		const auto& chain = *robot.second->GetChain();
		const auto& home = robot.second->GetHomeConfiguration();
		auto analyze_topology = Model::TopologyAnalyzer::Analyze( chain, home );

		EXPECT_TRUE( CheckEquality( robot.second->GetTopology(),  analyze_topology ) )
		    << "Analyze fail for robot " << robot.first << "Failed";
	}
}

// ------------------------------------------------------------


}