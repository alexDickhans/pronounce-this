#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> OPSkills1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(24.5198_in, 10.2825_in),
PathPlanner::Point(31.6384_in, 29.661_in),
PathPlanner::Point(45.0847_in, 38.1638_in),
PathPlanner::Point(45.2825_in, 65.452_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(45.2825_in, 65.452_in),
PathPlanner::Point(52.4011_in, 48.0508_in),
PathPlanner::Point(68.8136_in, 52.8955_in),
PathPlanner::Point(75.1412_in, 34.6045_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(69.6045_in, 34.4068_in),
PathPlanner::Point(69.6045_in, 49.1384_in),
PathPlanner::Point(69.3079_in, 60.0636_in),
PathPlanner::Point(69.4068_in, 73.5594_in)
,false),
new SinusoidalVelocityProfile(0.0, 45_in/second, 80_in/second/second, 0.0)},
{PathPlanner::BezierSegment(
PathPlanner::Point(69.4068_in, 66.8362_in),
PathPlanner::Point(73.4604_in, 54.428_in),
PathPlanner::Point(94.8658_in, 60.3355_in),
PathPlanner::Point(100.057_in, 44.887_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(93.3333_in, 35.1977_in),
PathPlanner::Point(93.3828_in, 40.512_in),
PathPlanner::Point(93.1603_in, 44.1825_in),
PathPlanner::Point(93.1356_in, 49.0396_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(93.1356_in, 52.7966_in),
PathPlanner::Point(93.3086_in, 63.7835_in),
PathPlanner::Point(93.2221_in, 74.0104_in),
PathPlanner::Point(93.1356_in, 80.8757_in)
,false),
new SinusoidalVelocityProfile(0.0, 30_in/second, 80_in/second/second, 0.0)},
{PathPlanner::BezierSegment(
PathPlanner::Point(93.1356_in, 80.8757_in),
PathPlanner::Point(93.0491_in, 87.741_in),
PathPlanner::Point(92.7957_in, 103.588_in),
PathPlanner::Point(92.5424_in, 108.955_in)
,false),
nullptr},
};
// PathPlanner made path
