#include "tankPurePursuit.hpp"

namespace Pronounce {
	TankPurePursuit::TankPurePursuit(TankDrivetrain* drivetrain) : PurePursuit() {
		this->drivetrain = drivetrain;
	}

	TankPurePursuit::TankPurePursuit(TankDrivetrain* drivetrain, double lookaheadDistance) : PurePursuit(lookaheadDistance) {
		this->drivetrain = drivetrain;
	}

	TankPurePursuit::TankPurePursuit(TankDrivetrain* drivetrain, Odometry* odometry, double lookaheadDistance) : PurePursuit(odometry, lookaheadDistance) {
		this->drivetrain = drivetrain;
	}

	void TankPurePursuit::updateDrivetrain() {
		double velocity = 200;
		drivetrain->tankSteerVelocity(200 * (2 + this->getPointData().curvature * this->drivetrain->getTrackWidth()), 200 * (2 - this->getPointData().curvature * this->drivetrain->getTrackWidth()));
	}

	void TankPurePursuit::stop() {
		drivetrain->tankSteerVelocity(0, 0);
	}

	TankPurePursuit::~TankPurePursuit() {
	}
} // namepsace Pronounce