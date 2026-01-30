#pragma once

#include "Twist.hpp"
#include "Types.hpp"

namespace SOArm100::Kinematics
{
class MatrixExponential
{
public:
MatrixExponential( const Twist& twist, double theta );
~MatrixExponential();

[[nodiscard]] Mat4d Compute() const;
operator Mat4d () const;
Mat4d operator * ( const MatrixExponential& other ) const;
Mat4d operator * ( const Mat4d& matrix ) const;

private:
Twist twist_;
double theta_;

Mat3d ComputeRotation( double theta ) const;
Vec3d ComputeTranslation( double theta ) const;
};
}
