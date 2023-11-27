#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> OPSkills1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(29.4634_in, 10.8756_in),
PathPlanner::Point(36.582_in, 30.2541_in),
PathPlanner::Point(45.8757_in, 42.1186_in),
PathPlanner::Point(46.0735_in, 69.4068_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(46.0735_in, 69.209_in),
PathPlanner::Point(53.1921_in, 51.8078_in),
PathPlanner::Point(63.6724_in, 52.6978_in),
PathPlanner::Point(70_in, 34.4068_in)
,true),
new SinusoidalVelocityProfile(0.0, 61_in/second, 150_in/second/second, 0.0)},
{PathPlanner::BezierSegment(
PathPlanner::Point(69.6045_in, 34.4068_in),
PathPlanner::Point(69.6045_in, 49.1384_in),
PathPlanner::Point(69.3079_in, 83.5947_in),
PathPlanner::Point(69.4068_in, 97.0905_in)
,false),
nullptr},
};
// PathPlanner made path
