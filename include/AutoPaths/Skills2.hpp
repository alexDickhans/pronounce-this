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
PathPlanner::Point(5.93226_in, 46.4695_in),
PathPlanner::Point(3.65822_in, 55.9607_in),
PathPlanner::Point(3.36158_in, 69.8023_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(3.36158_in, 69.8023_in),
PathPlanner::Point(3.06494_in, 83.6438_in),
PathPlanner::Point(3.90535_in, 96.497_in),
PathPlanner::Point(5.33898_in, 105.989_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(5.53672_in, 105.989_in),
PathPlanner::Point(11.9139_in, 132.882_in),
PathPlanner::Point(32.5531_in, 135.749_in),
PathPlanner::Point(54.1809_in, 136.243_in)
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
PathPlanner::Point(26.8617_in, 135.18_in),
PathPlanner::Point(28.6568_in, 94.4828_in),
PathPlanner::Point(46.8644_in, 91.7515_in)
,false),
nullptr},
};
// PathPlanner made path
