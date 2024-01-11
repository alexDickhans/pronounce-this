#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, QSpeed>> TestPath = {{PathPlanner::BezierSegment(
PathPlanner::Point(10.678_in, 35_in),
PathPlanner::Point(5.14124_in, 52.4011_in),
PathPlanner::Point(4.9435_in, 83.0508_in),
PathPlanner::Point(10.8757_in, 99.661_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(10.8757_in, 99.661_in),
PathPlanner::Point(16.8079_in, 113.898_in),
PathPlanner::Point(26.6949_in, 109.251_in),
PathPlanner::Point(36.1864_in, 102.232_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(36.1864_in, 102.232_in),
PathPlanner::Point(45.678_in, 95.2119_in),
PathPlanner::Point(93.5311_in, 97.6342_in),
PathPlanner::Point(105.593_in, 104.011_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(105.593_in, 104.011_in),
PathPlanner::Point(117.655_in, 110.388_in),
PathPlanner::Point(128.333_in, 112.143_in),
PathPlanner::Point(134.661_in, 100.056_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(134.661_in, 100.056_in),
PathPlanner::Point(140.989_in, 87.9696_in),
PathPlanner::Point(140_in, 57.8266_in),
PathPlanner::Point(135.057_in, 41.1299_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(135.057_in, 41.1299_in),
PathPlanner::Point(130.706_in, 27.7948_in),
PathPlanner::Point(118.347_in, 26.6517_in),
PathPlanner::Point(105.791_in, 39.7458_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(105.791_in, 39.7458_in),
PathPlanner::Point(96.9915_in, 49.2805_in),
PathPlanner::Point(85.7698_in, 52.225_in),
PathPlanner::Point(71.9774_in, 52.0056_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(71.9774_in, 52.0056_in),
PathPlanner::Point(58.185_in, 51.7863_in),
PathPlanner::Point(46.197_in, 43.7886_in),
PathPlanner::Point(36.5819_in, 33.0226_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(36.5819_in, 33.0226_in),
PathPlanner::Point(26.9668_in, 22.2566_in),
PathPlanner::Point(13.5823_in, 25.069_in),
PathPlanner::Point(12.4576_in, 40.9322_in)
,false),
0.0},
};
// PathPlanner made path
