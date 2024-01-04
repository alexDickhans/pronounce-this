#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, QSpeed>> CloseAWP1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(120.028_in, 22.7401_in),
PathPlanner::Point(107.175_in, 43.3051_in),
PathPlanner::Point(84.2373_in, 41.9209_in),
PathPlanner::Point(69.209_in, 41.9209_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(69.209_in, 41.9209_in),
PathPlanner::Point(79.4915_in, 41.9209_in),
PathPlanner::Point(91.2571_in, 40.4379_in),
PathPlanner::Point(91.9492_in, 31.8362_in)
,true),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(91.9492_in, 31.8362_in),
PathPlanner::Point(92.4435_in, 44.3927_in),
PathPlanner::Point(92.2952_in, 53.1427_in),
PathPlanner::Point(92.3446_in, 64.0678_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(92.3446_in, 64.0678_in),
PathPlanner::Point(92.1963_in, 56.6031_in),
PathPlanner::Point(92.0727_in, 51.8326_in),
PathPlanner::Point(92.3446_in, 44.4915_in)
,true),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(92.3446_in, 44.4915_in),
PathPlanner::Point(92.6165_in, 53.7606_in),
PathPlanner::Point(88.328_in, 65.5385_in),
PathPlanner::Point(74.1525_in, 65.452_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(74.1525_in, 65.452_in),
PathPlanner::Point(85.2878_in, 64.97_in),
PathPlanner::Point(91.6834_in, 59.5754_in),
PathPlanner::Point(92.1469_in, 45.8757_in)
,true),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(92.1469_in, 45.8757_in),
PathPlanner::Point(92.2149_in, 54.9161_in),
PathPlanner::Point(92.1809_in, 61.9637_in),
PathPlanner::Point(92.1469_in, 70.3955_in)
,false),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(101.836_in, 66.4407_in),
PathPlanner::Point(101.802_in, 52.1323_in),
PathPlanner::Point(103.303_in, 43.9616_in),
PathPlanner::Point(105.593_in, 32.8249_in)
,true),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(105.593_in, 32.8249_in),
PathPlanner::Point(111.444_in, 14.7672_in),
PathPlanner::Point(130.764_in, 15.6887_in),
PathPlanner::Point(132.288_in, 33.0226_in)
,true),
0.0},
{PathPlanner::BezierSegment(
PathPlanner::Point(132.288_in, 33.0226_in),
PathPlanner::Point(133.812_in, 50.3565_in),
PathPlanner::Point(121.186_in, 53.2574_in),
PathPlanner::Point(120.424_in, 67.2316_in)
,true),
0.0},
};
// PathPlanner made path
