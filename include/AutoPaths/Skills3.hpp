#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Skills3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(46.0734_in, 81.4689_in),
PathPlanner::Point(50.0282_in, 94.7175_in),
PathPlanner::Point(57.9379_in, 111.327_in),
PathPlanner::Point(59.9152_in, 125.565_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(57.7401_in, 114.887_in),
PathPlanner::Point(56.5536_in, 100.452_in),
PathPlanner::Point(55.5649_in, 77.7122_in),
PathPlanner::Point(71.5819_in, 75.7345_in)
,false),
nullptr},
};
// PathPlanner made path
