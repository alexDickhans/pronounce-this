#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Skills6 = {{PathPlanner::BezierSegment(
PathPlanner::Point(95.3107_in, 129.322_in),
PathPlanner::Point(112.119_in, 128.927_in),
PathPlanner::Point(104.209_in, 112.119_in),
PathPlanner::Point(91.7514_in, 97.2881_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(91.7514_in, 97.2881_in),
PathPlanner::Point(84.0396_in, 89.774_in),
PathPlanner::Point(57.6413_in, 84.435_in),
PathPlanner::Point(57.5424_in, 73.5593_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(69.6045_in, 75.339_in),
PathPlanner::Point(69.3079_in, 88.983_in),
PathPlanner::Point(69.654_in, 112.119_in),
PathPlanner::Point(69.4068_in, 126.356_in)
,true),
nullptr},
};
// PathPlanner made path
