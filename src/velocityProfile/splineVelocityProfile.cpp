//
// Created by alex on 8/28/23.
//

#include "velocityProfile/splineVelocityProfile.hpp"

namespace Pronounce {
	SplineVelocityProfile::SplineVelocityProfile(ProfileConstraints profileConstraints, SplinePath &splinePath) : velocityProfile(SinusoidalVelocityProfile(splinePath.getPath(0.1).distance(), profileConstraints)) {

	}

	SplineVelocityProfile::SplineVelocityProfile(VelocityProfile &velocityProfile, SplinePath &splinePath) : velocityProfile(velocityProfile), splinePath(splinePath) {

	}
} // Pronounce