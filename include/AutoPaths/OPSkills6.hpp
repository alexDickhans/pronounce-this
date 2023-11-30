#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> OPSkills6 = {{PathPlanner::BezierSegment(
PathPlanner::Point(40.9322_in, 132.486_in),
PathPlanner::Point(26.6949_in, 131.893_in),
PathPlanner::Point(25.9039_in, 122.4_in),
PathPlanner::Point(10.6779_in, 119.039_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(10.8757_in, 111.328_in),
PathPlanner::Point(25.1129_in, 97.2879_in),
PathPlanner::Point(81.1723_in, 100.254_in),
PathPlanner::Point(99.6613_in, 115.084_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(100.453_in, 99.6607_in),
PathPlanner::Point(130.41_in, 117.458_in),
PathPlanner::Point(127.394_in, 134.265_in),
PathPlanner::Point(101.045_in, 134.463_in)
,true),
nullptr},
};
// PathPlanner made path
