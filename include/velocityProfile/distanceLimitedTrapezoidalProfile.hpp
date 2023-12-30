#pragma once

#include "velocityProfile.hpp"
#include "utils/linearInterpolator.hpp"
#include "Eigen/Eigen"

namespace Pronounce {
	class DistanceLimitedTrapezoidalProfile : public VelocityProfile {
	private:
		QLength length{0.0};
		QTime time{0.0};
		LinearInterpolator timeToSpeed;
		LinearInterpolator timeToDistance;
	public:
		DistanceLimitedTrapezoidalProfile(std::vector<Eigen::Vector2d> distanceVelocity, QLength distance, ProfileConstraints profileConstraints) {

		}
	};
}