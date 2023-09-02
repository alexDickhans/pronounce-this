//
// Created by alex on 8/28/23.
//

#ifndef PRONOUNCE_THIS_SPLINEVELOCITYPROFILE_HPP
#define PRONOUNCE_THIS_SPLINEVELOCITYPROFILE_HPP

#include "velocityProfile.hpp"
#include "sinusoidalVelocityProfile.hpp"
#include "utils/splinePath.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include "units/units.hpp"

namespace Pronounce {

	class SplineVelocityProfile {
	private:
		VelocityProfile& velocityProfile;
		ProfileConstraints profileConstraints;
		SplinePath& splinePath;
		Path builtSplinePath;
		int pathGranularity{20};
	public:
		SplineVelocityProfile() = delete;
		SplineVelocityProfile(VelocityProfile& velocityProfile, SplinePath& splinePath);
		SplineVelocityProfile(ProfileConstraints profileConstraints, SplinePath& splinePath);

		void build() {
			builtSplinePath = splinePath.getPath(1.0/(double) pathGranularity);
			velocityProfile.setDistance(builtSplinePath.distance());
			velocityProfile.setProfileConstraints({profileConstraints.maxVelocity, profileConstraints.maxAcceleration, profileConstraints.maxJerk});
		}

		std::pair<TankChassisSpeeds, Angle> getChassisSpeedsAtTime(QTime time) {
			TankChassisSpeeds chassisSpeeds;
			chassisSpeeds.speed = velocityProfile.getVelocityByTime(time);
			QLength distance = velocityProfile.getDistanceByTime(time);

			double t = builtSplinePath.getTValueByDistance(distance)/(double) pathGranularity;

			chassisSpeeds.curvature = splinePath.getCurvatureAtT(t);

			return {chassisSpeeds, splinePath.getOrientation(t)};
		}
	};

} // Pronounce

#endif //PRONOUNCE_THIS_SPLINEVELOCITYPROFILE_HPP
