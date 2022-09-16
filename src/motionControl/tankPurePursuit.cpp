#include "tankPurePursuit.hpp"

namespace Pronounce {

	TankPurePursuit::TankPurePursuit(AbstractTankDrivetrain* drivetrain, ContinuousOdometry* odometry, PurePursuitProfile currentProfile) : PurePursuit(odometry, currentProfile) {
		this->drivetrain = drivetrain;
	}

	void TankPurePursuit::updateDrivetrain(PurePursuitPointData pointData) {

		if (isDone(this->getStopDistance())) {
			this->stop();
			return;
		}

		double side = sqrt(abs(clamp(pointData.localLookaheadVector.getCartesian().getY().getValue() / this->getCurrentProfile().lookaheadDistance.getValue(), -1.0, 1.0))) * signnum_c(pointData.localLookaheadVector.getCartesian().getY().getValue());

		side = abs(side) < 0.5 ? signnum_c(side) : side;

		QSpeed speed = this->getCurrentProfile().velocityProfile.getVelocityByDistance(pointData.distanceFromBeginning).getValue() * side;

		double motorSpeed = clamp(clamp(speed.getValue(), -speed.getValue(), speed.getValue()), -this->getSpeed().getValue(), this->getSpeed().getValue());

		drivetrain->driveCurvature(motorSpeed, pointData.curvature);
	}

	void TankPurePursuit::stop() {
		drivetrain->tankSteerVelocity(0.0, 0.0);
	}

	TankPurePursuit::~TankPurePursuit() {
	}
} // namepsace Pronounce