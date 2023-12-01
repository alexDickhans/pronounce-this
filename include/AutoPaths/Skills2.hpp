#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Skills2 = {{PathPlanner::BezierSegment(
PathPlanner::Point(37.3729_in, 10.4802_in),
PathPlanner::Point(20.5649_in, 10.8757_in),
PathPlanner::Point(12.0621_in, 17.7968_in),
PathPlanner::Point(8.70058_in, 31.8365_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(8.50284_in, 31.8365_in),
PathPlanner::Point(5.93226_in, 46.4695_in),
PathPlanner::Point(3.65822_in, 55.9607_in),
PathPlanner::Point(3.36158_in, 69.8023_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(3.36158_in, 69.8023_in),
PathPlanner::Point(3.06494_in, 83.6438_in),
PathPlanner::Point(4.69631_in, 96.497_in),
PathPlanner::Point(6.12994_in, 105.989_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(6.32768_in, 105.791_in),
PathPlanner::Point(11.9139_in, 128.729_in),
PathPlanner::Point(32.9485_in, 135.353_in),
PathPlanner::Point(54.5763_in, 135.847_in)
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
PathPlanner::Point(34.1781_in, 135.378_in),
PathPlanner::Point(38.7416_in, 77.0816_in),
PathPlanner::Point(52.9944_in, 75.339_in)
,false),
nullptr},
};
// PathPlanner made path
