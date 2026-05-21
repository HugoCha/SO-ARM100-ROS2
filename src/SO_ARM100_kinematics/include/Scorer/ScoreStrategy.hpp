#pragma once

#include <sys/types.h>

namespace  SOArm100::Kinematics::Scorer
{

enum class ScoreStrategy : u_int8_t
{
Manipulability = 1,
Centered       = 2,
CloseToLast    = 4,
CloseToSeed    = 8,
};

inline ScoreStrategy operator|( ScoreStrategy strat1, ScoreStrategy strat2 )
{
    return static_cast< ScoreStrategy >( 
        static_cast< u_int8_t >( strat1 ) | 
        static_cast< u_int8_t >( strat2 ) );
}

}