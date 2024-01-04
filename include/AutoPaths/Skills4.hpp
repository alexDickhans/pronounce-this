#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, QSpeed>> Skills4 = {{PathPlanner::BezierSegment(
PathPlanner::Point(72.7684_in, 78.7005_in),
PathPlanner::Point(70.1977_in, 90.1695_in),
PathPlanner::Point(68.0226_in, 101.045_in),
PathPlanner::Point(68.6158_in, 122.204_in)
,true),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(69.8023_in, 116.272_in),
PathPlanner::Point(69.4069_in, 77.7124_in),
PathPlanner::Point(103.122_in, 87.6979_in),
PathPlanner::Point(123.39_in, 85.0283_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(105.198_in, 78.8983_in),
PathPlanner::Point(95.8051_in, 90.2682_in),
PathPlanner::Point(91.8008_in, 101.588_in),
PathPlanner::Point(87.7966_in, 122.994_in)
,true),
0.0},
};
// PathPlanner made path
