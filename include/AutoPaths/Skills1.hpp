#pragma once
#include <vector>
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
using namespace Pronounce;
std::vector<std::pair<PathPlanner::BezierSegment, QSpeed>> Skills1 = {{PathPlanner::BezierSegment(
PathPlanner::Point(15.226_in, 30.2542_in),
PathPlanner::Point(27.0904_in, 30.2542_in),
PathPlanner::Point(24.9154_in, 22.7401_in),
PathPlanner::Point(22.5424_in, 5.33898_in)
,true),
0.0},
};
// PathPlanner made path
