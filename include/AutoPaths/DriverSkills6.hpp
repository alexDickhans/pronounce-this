#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, Eigen::Vector3d>> DriverSkills6 = {{PathPlanner::BezierSegment(
PathPlanner::Point(59.9153_in, 64.8587_in),
PathPlanner::Point(70.1977_in, 65.2542_in),
PathPlanner::Point(93.1356_in, 65.452_in),
PathPlanner::Point(110.734_in, 65.8475_in)
,false),
{}},
{PathPlanner::BezierSegment(
PathPlanner::Point(110.735_in, 60.9039_in),
PathPlanner::Point(96.6455_in, 56.0593_in),
PathPlanner::Point(110.018_in, 43.1568_in),
PathPlanner::Point(115.876_in, 25.5086_in)
,true),
{}},
{PathPlanner::BezierSegment(
PathPlanner::Point(110.536_in, 23.9266_in),
PathPlanner::Point(114.021_in, 14.9788_in),
PathPlanner::Point(122.759_in, 12.0375_in),
PathPlanner::Point(129.52_in, 25.5085_in)
,true),
{}},
{PathPlanner::BezierSegment(
PathPlanner::Point(129.52_in, 25.7062_in),
PathPlanner::Point(134.698_in, 40.9569_in),
PathPlanner::Point(135.174_in, 55.6762_in),
PathPlanner::Point(135.057_in, 70.3955_in)
,true),
{}},
{PathPlanner::BezierSegment(
PathPlanner::Point(135.057_in, 70.3955_in),
PathPlanner::Point(134.939_in, 85.1148_in),
PathPlanner::Point(136.777_in, 112.459_in),
PathPlanner::Point(125.763_in, 123.983_in)
,true),
{}},
{PathPlanner::BezierSegment(
PathPlanner::Point(125.763_in, 123.983_in),
PathPlanner::Point(115.539_in, 132.146_in),
PathPlanner::Point(106.414_in, 132.909_in),
PathPlanner::Point(96.1017_in, 132.684_in)
,true),
{}},
};
// PathPlanner made path
