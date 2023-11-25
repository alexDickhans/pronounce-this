#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> OPSkills6 = {{PathPlanner::BezierSegment(
PathPlanner::Point(40.9322_in, 132.486_in),
PathPlanner::Point(26.6949_in, 131.893_in),
PathPlanner::Point(24.9152_in, 115.282_in),
PathPlanner::Point(9.68922_in, 111.921_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(10.8757_in, 111.328_in),
PathPlanner::Point(28.87_in, 105.593_in),
PathPlanner::Point(79.9859_in, 93.7288_in),
PathPlanner::Point(102.232_in, 101.045_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(102.232_in, 101.045_in),
PathPlanner::Point(139.11_in, 114.294_in),
PathPlanner::Point(127.592_in, 131.299_in),
PathPlanner::Point(101.243_in, 131.497_in)
,false),
nullptr},
};
// PathPlanner made path
