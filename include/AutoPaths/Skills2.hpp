#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Skills2 = {{PathPlanner::BezierSegment(
PathPlanner::Point(37.3729_in, 10.4802_in),
PathPlanner::Point(20.5649_in, 10.8757_in),
PathPlanner::Point(13.4463_in, 17.7968_in),
PathPlanner::Point(10.0848_in, 31.8365_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(8.50284_in, 31.8365_in),
PathPlanner::Point(6.52548_in, 46.6672_in),
PathPlanner::Point(6.62432_in, 55.9607_in),
PathPlanner::Point(6.32768_in, 69.8023_in)
,true),
new SinusoidalVelocityProfile(0.0, 37_in/second, 100_in/second/second, 0.0)},
{PathPlanner::BezierSegment(
PathPlanner::Point(5.53672_in, 69.8023_in),
PathPlanner::Point(5.24008_in, 83.6438_in),
PathPlanner::Point(4.30083_in, 96.497_in),
PathPlanner::Point(5.73446_in, 105.989_in)
,true),
new SinusoidalVelocityProfile(0.0, 37_in/second, 100_in/second/second, 0.0)},
{PathPlanner::BezierSegment(
PathPlanner::Point(5.53672_in, 105.989_in),
PathPlanner::Point(11.9139_in, 129.718_in),
PathPlanner::Point(32.3554_in, 131.003_in),
PathPlanner::Point(53.9832_in, 131.497_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(35.3955_in, 135.254_in),
PathPlanner::Point(32.3058_in, 135.155_in),
PathPlanner::Point(29.006_in, 135.304_in),
PathPlanner::Point(24.9153_in, 135.254_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(24.9153_in, 135.254_in),
PathPlanner::Point(30.7115_in, 135.205_in),
PathPlanner::Point(37.206_in, 135.131_in),
PathPlanner::Point(45.678_in, 135.057_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(45.678_in, 135.057_in),
PathPlanner::Point(27.2571_in, 134.983_in),
PathPlanner::Point(49.8151_in, 105.161_in),
PathPlanner::Point(47.0622_in, 84.6329_in)
,false),
nullptr},
};
// PathPlanner made path
