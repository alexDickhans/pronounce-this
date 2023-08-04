#include "tankPurePursuit.hpp"

namespace Pronounce {

	TankPurePursuit::TankPurePursuit(std::string name, AbstractTankDrivetrain* drivetrain, ContinuousOdometry* odometry, PurePursuitProfile currentProfile, Path path) : PurePursuit(name, odometry, currentProfile, path) {
		this->drivetrain = drivetrain;
	}

	void TankPurePursuit::updateDrivetrain(PurePursuitPointData pointData) {
		std::cout << "Tank" << std::endl;

		if (isDone(this->getStopDistance())) {
			this->stop();
			return;
		}

		double side = sqrt(abs(clamp(pointData.localLookaheadVector.getCartesian().getY().getValue() / this->getCurrentProfile().lookaheadDistance.getValue(), -1.0, 1.0))) * signnum_c(pointData.localLookaheadVector.getCartesian().getY().getValue());

		side = abs(side) < 0.5 ? signnum_c(side) : side;

		QAcceleration maxAcceleration = 200_in/second/second;

		QSpeed speed = clamp(this->getCurrentProfile().velocityProfile.getProfileConstraints().maxVelocity, -this->timeSinceStart() * maxAcceleration, this->timeSinceStart() * maxAcceleration);

		speed = std::max(std::min(speed.getValue(), (2 * maxAcceleration*(pointData.distanceFromEnd-1_in)).getValue()), (1_in/second).getValue()) * side;

		std::cout << "MaxEndAcceleration: " << (2 * maxAcceleration*pointData.distanceFromEnd).getValue() << std::endl;

		drivetrain->driveCurvature(speed, pointData.curvature);
	}

	void TankPurePursuit::stop() {
		drivetrain->tankSteerVelocity(0.0, 0.0);
	}

	TankPurePursuit::~TankPurePursuit() {
	}
} // namepsace Pronounce