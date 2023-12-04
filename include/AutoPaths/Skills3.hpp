#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Skills3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(46.0734_in, 93.1355_in),
PathPlanner::Point(53.5876_in, 104.605_in),
PathPlanner::Point(55.9605_in, 111.722_in),
PathPlanner::Point(57.5423_in, 126.158_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(57.7401_in, 114.887_in),
PathPlanner::Point(56.5536_in, 100.452_in),
PathPlanner::Point(60.9039_in, 77.1189_in),
PathPlanner::Point(81.6666_in, 75.339_in)
,false),
nullptr},
};
// PathPlanner made path
