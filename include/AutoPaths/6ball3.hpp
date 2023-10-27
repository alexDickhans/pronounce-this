#pragma once
#include <vector>
std::vector<std::pair<PathPlanner::BezierSegment, Pronounce::SinusoidalVelocityProfile*>> Auto6BallPath3 = {{PathPlanner::BezierSegment(
PathPlanner::Point(93_in, 90_in),
PathPlanner::Point(93_in, 100_in),
PathPlanner::Point(82.2599_in, 99.2655_in),
PathPlanner::Point(81.6667_in, 115.48_in)
,false),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(81.6667_in, 115.48_in),
PathPlanner::Point(82.2599_in, 105.395_in),
PathPlanner::Point(98.4746_in, 104.407_in),
PathPlanner::Point(110.141_in, 104.407_in)
,true),
nullptr},
{PathPlanner::BezierSegment(
PathPlanner::Point(98.4746_in, 104.407_in),
PathPlanner::Point(89.9717_in, 101.045_in),
PathPlanner::Point(84.0396_in, 101.836_in),
PathPlanner::Point(83.6441_in, 85.226_in)
,false),
nullptr},
};
// PathPlanner made path
