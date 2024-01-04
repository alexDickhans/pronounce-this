#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, QSpeed>> Auto6BallElim3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(55.565_in, 46.4689_in),
PathPlanner::Point(49.6328_in, 51.0169_in),
PathPlanner::Point(46.2712_in, 54.774_in),
PathPlanner::Point(45.678_in, 63.0791_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(45.678_in, 63.0791_in),
PathPlanner::Point(45.8757_in, 39.7458_in),
PathPlanner::Point(15.9181_in, 19.2797_in),
PathPlanner::Point(8.8983_in, 37.1751_in)
,true),
0.0},
};
// PathPlanner made path
