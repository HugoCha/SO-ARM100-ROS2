#pragma once

namespace SOArm100::Kinematics 
{
class AdaptativeDamping
{
public:
  virtual double Damping() const = 0;
};
}