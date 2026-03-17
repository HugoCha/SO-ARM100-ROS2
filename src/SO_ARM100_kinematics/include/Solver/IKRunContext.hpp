#pragma once

#include <stop_token>

namespace SOArm100::Kinematics::Solver
{
class IKRunContext
{
public:
IKRunContext() : 
    stop_source_(),
    stop_token_( stop_source_.get_token() )
{}

bool StopRequested() const {
    return stop_token_.stop_requested();
}

void RequestStop() const {
    stop_source_.request_stop();
}

private:
std::stop_source stop_source_;
std::stop_token stop_token_;
};
}