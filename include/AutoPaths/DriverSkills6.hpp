#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> DriverSkills6 = {{PathPlanner::BezierSegment(
PathPlanner::Point(111.13_in, 54.774_in),
PathPlanner::Point(111.476_in, 44.5904_in),
PathPlanner::Point(111.402_in, 34.8517_in),
PathPlanner::Point(114.887_in, 25.5086_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(110.536_in, 23.9266_in),
PathPlanner::Point(116.592_in, 15.1765_in),
PathPlanner::Point(123.55_in, 12.0375_in),
PathPlanner::Point(130.311_in, 25.5085_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(129.52_in, 25.7062_in),
PathPlanner::Point(134.698_in, 40.9569_in),
PathPlanner::Point(137.942_in, 55.2807_in),
PathPlanner::Point(137.825_in, 70_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(137.825_in, 70_in),
PathPlanner::Point(137.707_in, 84.7193_in),
PathPlanner::Point(136.777_in, 112.459_in),
PathPlanner::Point(125.763_in, 123.983_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(125.763_in, 123.983_in),
PathPlanner::Point(115.539_in, 132.146_in),
PathPlanner::Point(106.414_in, 132.909_in),
PathPlanner::Point(96.1017_in, 132.684_in)
,true),
nullptr},
};
// PathPlanner made path
