#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Eigen::Vector3d>> Skills6 = {{PathPlanner::BezierSegment(
PathPlanner::Point(95.3107_in, 129.322_in),
PathPlanner::Point(112.119_in, 128.927_in),
PathPlanner::Point(104.209_in, 112.119_in),
PathPlanner::Point(91.7514_in, 97.2881_in)
,false),
{}},
{PathPlanner::BezierSegment(
PathPlanner::Point(91.7514_in, 97.2881_in),
PathPlanner::Point(84.0396_in, 89.774_in),
PathPlanner::Point(57.6413_in, 84.435_in),
PathPlanner::Point(57.5424_in, 73.5593_in)
,false),
{}},
{PathPlanner::BezierSegment(
PathPlanner::Point(69.6045_in, 75.339_in),
PathPlanner::Point(69.3079_in, 86.2146_in),
PathPlanner::Point(69.654_in, 112.119_in),
PathPlanner::Point(69.4068_in, 126.356_in)
,true),
{}},
{PathPlanner::BezierSegment(
PathPlanner::Point(69.4068_in, 126.356_in),
PathPlanner::Point(69.1596_in, 92.5422_in),
PathPlanner::Point(16.8821_in, 127.74_in),
PathPlanner::Point(9.88701_in, 96.4972_in)
,false),
{}},
{PathPlanner::BezierSegment(
PathPlanner::Point(9.88701_in, 96.4972_in),
PathPlanner::Point(8.42868_in, 86.808_in),
PathPlanner::Point(8.76236_in, 79.2938_in),
PathPlanner::Point(8.70057_in, 71.1864_in)
,false),
{}},
};
// PathPlanner made path
