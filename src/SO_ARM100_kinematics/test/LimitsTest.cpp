#include "Model/Limits.hpp"

#include <gtest/gtest.h>
#include <random_numbers/random_numbers.h>
#include <limits>

namespace SOArm100::Kinematics::Test
{

// ------------------------------------------------------------
// ------------------------------------------------------------

class LimitsTest : public ::testing::Test
{
protected:
void SetUp() override {
}
void TearDown() override {
}
};

// ------------------------------------------------------------
// ------------------------------------------------------------

TEST_F( LimitsTest, DefaultConstructor )
{
	Model::Limits limits;
	ASSERT_EQ( limits.Min(), -std::numeric_limits< double >::infinity() );
	ASSERT_EQ( limits.Max(), std::numeric_limits< double >::infinity() );
}

// ------------------------------------------------------------

TEST_F( LimitsTest, ConstructorWithMinMax )
{
	double min = -10.0;
	double max = 10.0;
	Model::Limits limits( min, max );
	ASSERT_EQ( limits.Min(), min );
	ASSERT_EQ( limits.Max(), max );
}

// ------------------------------------------------------------

TEST_F( LimitsTest, ConstructorWithInvalidMinMax )
{
	ASSERT_THROW( Model::Limits limits( 10.0, -10.0 ), std::invalid_argument ); // Should assert and die
}

// ------------------------------------------------------------

TEST_F( LimitsTest, WithinFunction )
{
	Model::Limits limits( -10.0, 10.0 );

	// Test within range
	ASSERT_TRUE( limits.Within( 0.0 ) );
	ASSERT_TRUE( limits.Within( -10.0 ) );
	ASSERT_TRUE( limits.Within( 10.0 ) );

	// Test outside range
	ASSERT_FALSE( limits.Within( -10.1 ) );
	ASSERT_FALSE( limits.Within( 10.1 ) );
}

// ------------------------------------------------------------

TEST_F( LimitsTest, ClampFunction )
{
	Model::Limits limits( -10.0, 10.0 );

	// Test within range
	ASSERT_EQ( limits.Clamp( 0.0 ), 0.0 );
	ASSERT_EQ( limits.Clamp( -10.0 ), -10.0 );
	ASSERT_EQ( limits.Clamp( 10.0 ), 10.0 );

	// Test outside range
	ASSERT_EQ( limits.Clamp( -10.1 ), -10.0 );
	ASSERT_EQ( limits.Clamp( 10.1 ), 10.0 );
}

// ------------------------------------------------------------

TEST_F( LimitsTest, RandomFunctionWithNullPointer )
{
	Model::Limits limits( -10.0, 10.0 );
	random_numbers::RandomNumberGenerator rng;
	ASSERT_THROW( limits.Random( rng, nullptr ), std::invalid_argument );
}

// ------------------------------------------------------------

TEST_F( LimitsTest, RandomFunction )
{
	Model::Limits limits( -10.0, 10.0 );
	random_numbers::RandomNumberGenerator rng;
	double random;

	for ( int i = 0; i < 100; ++i )
	{
		limits.Random( rng, & random );
		ASSERT_TRUE( limits.Within( random ) );
	}
}

// ------------------------------------------------------------

}
