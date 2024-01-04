#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, QSpeed>> Skills2 = {{PathPlanner::BezierSegment(
PathPlanner::Point(42.9097_in, 3.75708_in),
PathPlanner::Point(26.1017_in, 4.15258_in),
PathPlanner::Point(11.6666_in, 17.5991_in),
PathPlanner::Point(8.30514_in, 31.6388_in)
,true),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(8.3051_in, 31.6388_in),
PathPlanner::Point(5.1413_in, 46.4695_in),
PathPlanner::Point(2.0763_in, 71.7799_in),
PathPlanner::Point(0.59322_in, 85.8192_in)
,true),
37_in/second},
{PathPlanner::BezierSegment(
PathPlanner::Point(4.54802_in, 64.8588_in),
PathPlanner::Point(2.47172_in, 83.4461_in),
PathPlanner::Point(5.28953_in, 96.6947_in),
PathPlanner::Point(6.72316_in, 106.187_in)
,true),
37_in/second},
{PathPlanner::BezierSegment(
PathPlanner::Point(6.72316_in, 106.187_in),
PathPlanner::Point(13.1003_in, 129.916_in),
PathPlanner::Point(31.7622_in, 138.517_in),
PathPlanner::Point(53.39_in, 139.011_in)
,true),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(35.3955_in, 135.254_in),
PathPlanner::Point(32.3058_in, 135.155_in),
PathPlanner::Point(29.006_in, 135.304_in),
PathPlanner::Point(24.9153_in, 135.254_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(24.9153_in, 135.254_in),
PathPlanner::Point(30.7115_in, 135.205_in),
PathPlanner::Point(37.206_in, 135.131_in),
PathPlanner::Point(45.678_in, 135.057_in)
,true),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(45.678_in, 135.057_in),
PathPlanner::Point(27.2571_in, 134.983_in),
PathPlanner::Point(49.8151_in, 105.161_in),
PathPlanner::Point(47.0622_in, 84.6329_in)
,false),
0.0},
};
// PathPlanner made path
