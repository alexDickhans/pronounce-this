#include "tankPurePursuit.hpp"

namespace Pronounce {
	TankPurePursuit::TankPurePursuit(AbstractTankDrivetrain* drivetrain) : PurePursuit() {
		this->drivetrain = drivetrain;
	}

	TankPurePursuit::TankPurePursuit(AbstractTankDrivetrain* drivetrain, double lookaheadDistance) : PurePursuit(lookaheadDistance) {
		this->drivetrain = drivetrain;
	}

	TankPurePursuit::TankPurePursuit(AbstractTankDrivetrain* drivetrain, Odometry* odometry, double lookaheadDistance) : PurePursuit(odometry, lookaheadDistance) {
		this->drivetrain = drivetrain;
	}

	void TankPurePursuit::updateDrivetrain() {

		if (!this->isEnabled()) {
			return;
		}

		if (isDone(this->getStopDistance())) {
			return;
		}

		PurePursuitPointData pointData = this->getPointData();

		double side = clamp(pointData.localLookaheadVector.getCartesian().getY() / this->getLookahead(), -1, 1);

		side = side < 0.3 ? 1 : side;

		double scalar = pointData.lookaheadVector.getMagnitude() / this->getLookahead();

		double speed = clamp(this->getSpeed() * side * scalar, -this->getSpeed(), this->getSpeed());

		std::cout << "Curvature: " << pointData.curvature << std::endl;
		std::cout << "Speed: " << speed << std::endl;

		// Drive backwards
		if (speed < 0) {
			pointData.curvature = -pointData.curvature;
		}

		drivetrain->driveCurvature(speed, pointData.curvature);
	}

	void TankPurePursuit::stop() {
		drivetrain->tankSteerVelocity(0, 0);
	}

	TankPurePursuit::~TankPurePursuit() {
	}
} // namepsace Pronounce