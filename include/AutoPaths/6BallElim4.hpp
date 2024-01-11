#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Auto6BallElim4 = {{PathPlanner::BezierSegment(
PathPlanner::Point(49.8305_in, 54.1807_in),
PathPlanner::Point(47.0622_in, 59.9152_in),
PathPlanner::Point(46.4689_in, 60.9039_in),
PathPlanner::Point(45.8757_in, 69.209_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(45.678_in, 63.0791_in),
PathPlanner::Point(45.4802_in, 47.2599_in),
PathPlanner::Point(17.8955_in, 42.2176_in),
PathPlanner::Point(7.90972_in, 53.1921_in)
,true),
nullptr},
};
// PathPlanner made path
