#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Skills3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(46.0734_in, 81.4689_in),
PathPlanner::Point(50.0282_in, 94.7175_in),
PathPlanner::Point(55.9605_in, 111.722_in),
PathPlanner::Point(57.5423_in, 126.158_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(57.7401_in, 114.887_in),
PathPlanner::Point(56.5536_in, 100.452_in),
PathPlanner::Point(62.4858_in, 78.8986_in),
PathPlanner::Point(78.5028_in, 76.9209_in)
,false),
nullptr},
};
// PathPlanner made path
