#pragma once

#include <algorithm>
#include <random_numbers/random_numbers.h>

namespace SOArm100::Kinematics
{
class Limits
{
public:
Limits() :
	Limits( -std::numeric_limits< double >::infinity(),
	        std::numeric_limits< double >::infinity() )
{
}

Limits( double min, double max ) :
	min_( min ),
	max_( max )
{
	assert( min <= max );
}

[[nodiscard]] double Min() const {
	return min_;
}

[[nodiscard]] double Max() const {
	return max_;
}

[[nodiscard]] bool Within( double value ) const {
	return value >= min_ && value <= max_;
}

[[nodiscard]] double Clamp( double value ) const {
	return std::clamp( value, min_, max_ );
}

void Random( random_numbers::RandomNumberGenerator& rng, double* random ) const {
	random[0] = rng.uniformReal( min_, max_ );
}

private:
double min_;
double max_;
};
}